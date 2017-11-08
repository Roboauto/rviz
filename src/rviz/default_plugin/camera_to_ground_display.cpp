/*
 * AerialMapDisplay.cpp
 *
 *  Copyright (c) 2014 Gaeth Cross. Apache 2 License.
 *
 *  This file is part of rviz_satellite.
 *
 *	Created on: 07/09/2014
 */

#include <QtGlobal>
#include <QImage>

#include <ros/ros.h>

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreImageCodec.h>
#include <OGRE/OgreVector3.h>

#include <std_msgs/String.h>

#include <boost/algorithm/string/erase.hpp>
#include <boost/foreach.hpp>
#include <boost/shared_ptr.hpp>

#include <image_transport/subscriber_plugin.h>

#include <pluginlib/class_loader.h>

#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/grid.h"
#include "rviz/properties/enum_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/tf_frame_property.h"
#include "rviz/properties/quaternion_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/properties/vector_property.h"
#include "rviz/validate_floats.h"
#include "rviz/display_context.h"

#include "camera_to_ground_display.h"



Ogre::TexturePtr textureFromImage(const QImage &image,
                                  const std::string &name) {
    //  convert to 24bit rgb
    QImage converted = image.convertToFormat(QImage::Format_RGB888).mirrored();

    //  create texture
    Ogre::TexturePtr texture;
    Ogre::DataStreamPtr data_stream;
    data_stream.bind(new Ogre::MemoryDataStream((void *)converted.constBits(), converted.byteCount()));

    const Ogre::String res_group = Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME;
    Ogre::TextureManager &texture_manager = Ogre::TextureManager::getSingleton();
    //  swap byte order when going from QImage to Ogre
    texture = texture_manager.loadRawData(name, res_group, data_stream,
                                        converted.width(), converted.height(),
                                        Ogre::PF_B8G8R8, Ogre::TEX_TYPE_2D, 0);
    return texture;
}



inline QImage  cvMatToQImage( const cv::Mat &inMat )
{
    switch ( inMat.type() ) {
        case CV_8UC4: {
            QImage image( inMat.data, inMat.cols, inMat.rows, static_cast<int>(inMat.step), QImage::Format_ARGB32 );
            return image;
        }

        case CV_8UC3: {
            QImage image( inMat.data, inMat.cols, inMat.rows, static_cast<int>(inMat.step), QImage::Format_RGB888 );
            return image.rgbSwapped();
        }

        case CV_8UC1: {
#if QT_VERSION >= QT_VERSION_CHECK(5, 5, 0)
            QImage image( inMat.data, inMat.cols, inMat.rows, static_cast<int>(inMat.step), QImage::Format_Grayscale8 );
#else
            static QVector<QRgb>  sColorTable;
            if ( sColorTable.isEmpty() ) {
               sColorTable.resize( 256 );
               for ( int i = 0; i < 256; ++i ) {
                  sColorTable[i] = qRgb( i, i, i );
               }
            }
            QImage image( inMat.data,
                          inMat.cols, inMat.rows,
                          static_cast<int>(inMat.step),
                          QImage::Format_Indexed8 );
            image.setColorTable( sColorTable );
#endif
            return image;
        }
        default:
            qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
            break;
    }
    return QImage();
}


namespace rviz {

    CameraToGround_Display::CameraToGround_Display() : Display(),
                                                       image_sub_(),
                                                       map_id_(0),
                                                       scene_id_(0) {

        static unsigned int map_ids = 0;
        map_id_ = map_ids++; //  global counter of map ids

        alpha_property_ = new FloatProperty("Alpha",
                                            TEXTURE_ALPHA_INIT_,
                                            "Amount of transparency to apply to the map.",
                                            this,
                                            SLOT(updateAlpha()));
        alpha_ = alpha_property_->getValue().toFloat();
        alpha_property_->setMin(TEXTURE_ALPHA_MIN_);
        alpha_property_->setMax(TEXTURE_ALPHA_MAX_);
        alpha_property_->setShouldBeSaved(true);

        length_in_meters_ = new IntProperty("Texture length",
                                            TEXTURE_LENGTH_INIT_,
                                            "How far from car texture will be rendered",
                                            this,
                                            SLOT(updateTextureLength()) );
        length_in_meters_->setMin(TEXTURE_LENGTH_MIN_);
        length_in_meters_->setMax(TEXTURE_LENGTH_MAX_);
        length_in_meters_->setShouldBeSaved(true);
        texture_length_ = length_in_meters_->getValue().toInt();


        width_in_meters_ = new IntProperty("Texture width",
                                           TEXTURE_WIDTH_INIT_,
                                           "How wide from car texture will be rendered",
                                           this,
                                           SLOT(updateTextureWidth()) );
        width_in_meters_->setMin(TEXTURE_WIDTH_MIN_);
        width_in_meters_->setMax(TEXTURE_WIDTH_MAX_);
        width_in_meters_->setShouldBeSaved(true);
        texture_width_ = width_in_meters_->getValue().toInt();

        pixels_per_meter_ = new IntProperty("Pixels per meter",
                                            TEXTURE_PIXEL_PER_METER_INIT_,
                                            "How many pixels will be rendered per one meter of texture",
                                            this,
                                            SLOT(updatePixelPerMeter()) );
        pixels_per_meter_->setMin(TEXTURE_PIXEL_PER_METER_MIN_);
        pixels_per_meter_->setMax(TEXTURE_PIXEL_PER_METER_MAX_);
        pixels_per_meter_->setShouldBeSaved(true);
        texture_pixel_per_meter_ = pixels_per_meter_->getValue().toInt();


        camera_name_ = new RosTopicProperty("Camera name",
                                            "/robo/camera/image_rect_color",
                                            "sensor_msgs/Image",
                                            "Contains name of projected camera",
                                            this,
                                            SLOT(updateCameraName()) );

        transport_property_ = new EnumProperty("Transport Hint",
                                          "raw",
                                          "Preferred method of sending images.",
                                          this,
                                          SLOT( updateTopic() ));
        connect(transport_property_, SIGNAL( requestOptions( EnumProperty* )), this, SLOT( fillTransportOptionList( EnumProperty* )));

        frame_property_ = new TfFrameProperty("Camera frame", "camera",
                                              "tf frame of projected camera", this,
                                              nullptr, false, SLOT(updateCameraFrame()), this);

        origin_frame_property_ = new TfFrameProperty("Origin frame",
                                                     "ground",
                                                     "frame of car origin at the ground level",
                                                     this,
                                                     nullptr, false, SLOT(updateOriginFrame()), this);

        unreliable_property_ = new BoolProperty( "Unreliable",
                                                 false,
                                                 "Prefer UDP topic transport",
                                                 this,
                                                 SLOT( updateTopic() ));

        projectionTool_ = new Projection(update_nh_,
                                        frame_property_->getStdString(),
                                        origin_frame_property_->getStdString(),
                                        texture_length_,
                                        texture_width_,
                                        texture_pixel_per_meter_);

        it_ = new image_transport::ImageTransport(update_nh_);

        messages_received_ = 0;
    }

    CameraToGround_Display::~CameraToGround_Display() {
        unsubscribe();
        clear();
    }



    void CameraToGround_Display::onInitialize() {

        frame_property_->setFrameManager(context_->getFrameManager());
        origin_frame_property_->setFrameManager(context_->getFrameManager());
        scanForTransportSubscriberPlugins();
        ROS_INFO("Initialization done");
    }



    void CameraToGround_Display::onEnable() {
        subscribe();
    }



    void CameraToGround_Display::onDisable() {
      unsubscribe();
      clear();
    }


    void CameraToGround_Display::onNewImage(const sensor_msgs::ImageConstPtr& msg) {

        setStatus(StatusProperty::Ok, "Image", "Images received");

        cv::Mat img = cv::Mat(msg->height, msg->width, CV_8UC3, (char*)&*msg->data.begin() ).clone();

        projectionTool_->warp_image_to_bird_view(img, camera_image_);

        messages_received_++;
        setStatus(StatusProperty::Ok, "Image", QString::number(messages_received_) + " images received");

        assembleScene();
    }


    void CameraToGround_Display::onNewCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg) {
        camera_info_ = *msg;
        projectionTool_->set_camera_info(*msg);
    }


    void CameraToGround_Display::subscribe() {

        if (!isEnabled()) {
            return;
        }

        image_sub_.reset(new image_transport::SubscriberFilter());

        try {
            image_sub_.reset(new image_transport::SubscriberFilter());

            if (!camera_name_->getTopicStd().empty() && !transport_property_->getStdString().empty() ) {

                // Determine UDP vs TCP transport for user selection.
                if (unreliable_property_->getBool())
                {
                    image_sub_->subscribe(*it_, camera_name_->getTopicStd(), 1,
                                    image_transport::TransportHints(transport_property_->getStdString(), ros::TransportHints().unreliable()));
                }
                else{
                    image_sub_->subscribe(*it_, camera_name_->getTopicStd(), 1,
                                    image_transport::TransportHints(transport_property_->getStdString()));
                }

                image_sub_->registerCallback(boost::bind(&CameraToGround_Display::onNewImage, this, _1));
            }
            setStatus(StatusProperty::Ok, "Topic", "OK");
        }
        catch (ros::Exception& e)
        {
            setStatus(StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
        }
        catch (image_transport::Exception& e)
        {
            setStatus( StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what());
        }

        setStatus(StatusProperty::Warn, "Image", "No Image received");
        messages_received_ = 0;
//        //image_sub_ = it_->subscribe(camera_name_->getStdString(), 1, &CameraToGround_Display::onNewImage, this);
//        image_sub_->subscribe(*it_,
//                              camera_name_->getTopicStd(),
//                              1,
//                              image_transport::TransportHints(transport_property_->getStdString(),
//                                                              ros::TransportHints().unreliable()));

        QStringList query = camera_name_->getString().split("/");
        QString camera_info_string = "/";
        for (int i = 0 ; i < query.size()-1 ; i++) {
            camera_info_string.append(QString("%0/").arg(query[i]));
        }
        camera_info_string.append("camera_info");
        camera_info_sub_ = update_nh_.subscribe(camera_info_string.toStdString(), 1, &CameraToGround_Display::onNewCameraInfo, this);

        ROS_INFO("Subscribing.");
    }

    void CameraToGround_Display::unsubscribe() {

        image_sub_.reset(new image_transport::SubscriberFilter());
        camera_info_sub_.shutdown();
        ROS_INFO("Unsubscribing.");
    }



    void CameraToGround_Display::updateAlpha() {
        alpha_ = alpha_property_->getFloat();
        ROS_INFO("Changing alpha to %f", alpha_);
    }



    void CameraToGround_Display::updateTextureLength() {
        texture_length_ = (length_in_meters_->getValue().toInt()/2)*2;
        projectionTool_->set_xRange(texture_length_);
    }



    void CameraToGround_Display::updateTextureWidth() {
        texture_width_ = width_in_meters_->getValue().toInt();
        projectionTool_->set_yRange(texture_width_);
    }



    void CameraToGround_Display::updatePixelPerMeter() {
        texture_pixel_per_meter_ = (pixels_per_meter_->getValue().toInt()/2)*2;
        projectionTool_->set_pixelPerMeter(texture_pixel_per_meter_);
    }


    void CameraToGround_Display::updateCameraName() {
        delete projectionTool_;
        projectionTool_ = new Projection(update_nh_,
                                        frame_property_->getStdString(),
                                        origin_frame_property_->getStdString(),
                                        texture_length_,
                                        texture_width_,
                                        texture_pixel_per_meter_);
    }



    void CameraToGround_Display::updateCameraFrame () {
        delete projectionTool_;
        projectionTool_ = new Projection(update_nh_,
                                        frame_property_->getStdString(),
                                        origin_frame_property_->getStdString(),
                                        texture_length_,
                                        texture_width_,
                                        texture_pixel_per_meter_);
    }


    void CameraToGround_Display::updateOriginFrame () {

        ORIGIN_FRAME = origin_frame_property_->getStdString();
        projectionTool_->set_originFrame(ORIGIN_FRAME);
    }




    void CameraToGround_Display::clear() {
        clearGeometry();
    }



    void CameraToGround_Display::clearGeometry() {
        for (MapObject &obj : objects_) {

            //  destroy object
            scene_node_->detachObject(obj.object);
            scene_manager_->destroyManualObject(obj.object);

            //  destroy texture
            if (!obj.texture.isNull()) {
              Ogre::TextureManager::getSingleton().remove(obj.texture->getName());
            }

            //  destroy material
            if (!obj.material.isNull()) {
              Ogre::MaterialManager::getSingleton().remove(obj.material->getName());
            }
        }
        objects_.clear();
    }



    void CameraToGround_Display::update(float, float) {
        //  creates all geometry, if necessary
        //assembleScene();
        //  draw
        //context_->queueRender();
    }



    void CameraToGround_Display::assembleScene() {

        transformGroundTexture();

        clearGeometry();

        double texture_height = texture_length_;
        double texture_width = texture_width_;

        double x_offset = 0;
        double y_offset = -((float)texture_width_)/2;
        double z_offset = 0;

        // locad image and transofmamtion
        QImage texture_image;
        if ( camera_image_.rows > 0 && camera_image_.cols > 0) {
            texture_image = cvMatToQImage(camera_image_);
        } else {
            return;
        }

        tf::StampedTransform transform;
        try {
            listener_.lookupTransform(fixed_frame_.toUtf8().constData(), ORIGIN_FRAME, ros::Time(0), transform);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            return;
        }


        // prepare texture
        Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create("material",
                                                                                  Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
        material->setReceiveShadows(false);
        material->getTechnique(0)->setLightingEnabled(false);
        material->setDepthBias(-16.0f, 0.0f);
        material->setCullingMode(Ogre::CULL_NONE);
        material->setDepthWriteEnabled(false);

        //  create textureing unit
        Ogre::Pass *pass = material->getTechnique(0)->getPass(0);
        Ogre::TextureUnitState *tex_unit = nullptr;
        if (pass->getNumTextureUnitStates() > 0) {
            tex_unit = pass->getTextureUnitState(0);
        } else {
            tex_unit = pass->createTextureUnitState();
        }


        Ogre::TexturePtr texture = textureFromImage(texture_image, "texture");


        tex_unit->setTextureName(texture->getName());
        tex_unit->setTextureFiltering(Ogre::TFO_BILINEAR);

        //  create an object
        const std::string obj_name = "object";
        Ogre::ManualObject *obj = scene_manager_->createManualObject(obj_name);
        scene_node_->attachObject(obj);

        //  configure depth & alpha properties
        if (alpha_ >= 0.9998) {
            material->setDepthWriteEnabled(!draw_under_);
            material->setSceneBlending(Ogre::SBT_REPLACE);
        } else {
            material->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
            material->setDepthWriteEnabled(false);
        }

        if (draw_under_) {
            obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_3);
        } else {
            obj->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
        }

        tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
                                    Ogre::LBS_CURRENT, alpha_);


        cv::Point3f rotated_point;
        rotated_point = currentFrameToGroundRotMatrix_ * rotated_point;

        // Draw //
        obj->begin(material->getName(), Ogre::RenderOperation::OT_TRIANGLE_LIST);

        //  bottom left
        rotated_point = currentFrameToGroundRotMatrix_ * cv::Point3f(x_offset,
                                                                     y_offset,
                                                                     z_offset);
        obj->position(rotated_point.x + transform.getOrigin().getX(),
                      rotated_point.y + transform.getOrigin().getY(),
                      rotated_point.z + transform.getOrigin().getZ());
        obj->textureCoord(0.0f, 0.0f);
        obj->normal(0.0f, 0.0f, 1.0f);

        // top right
        rotated_point = currentFrameToGroundRotMatrix_ * cv::Point3f(x_offset + texture_height,
                                                                     y_offset + texture_width,
                                                                     z_offset);
        obj->position(rotated_point.x + transform.getOrigin().getX(),
                      rotated_point.y + transform.getOrigin().getY(),
                      rotated_point.z + transform.getOrigin().getZ());
        obj->textureCoord(1.0f, 1.0f);
        obj->normal(0.0f, 0.0f, 1.0f);

        // top left
        rotated_point = currentFrameToGroundRotMatrix_ * cv::Point3f(x_offset,
                                                                     y_offset + texture_width,
                                                                     z_offset);
        obj->position(rotated_point.x + transform.getOrigin().getX(),
                      rotated_point.y + transform.getOrigin().getY(),
                      rotated_point.z + transform.getOrigin().getZ());
        obj->textureCoord(0.0f, 1.0f);
        obj->normal(0.0f, 0.0f, 1.0f);

        //  bottom left
        rotated_point = currentFrameToGroundRotMatrix_ * cv::Point3f(x_offset,
                                                                     y_offset,
                                                                     z_offset);
        obj->position(rotated_point.x + transform.getOrigin().getX(),
                      rotated_point.y + transform.getOrigin().getY(),
                      rotated_point.z + transform.getOrigin().getZ());
        obj->textureCoord(0.0f, 0.0f);
        obj->normal(0.0f, 0.0f, 1.0f);

        // bottom right
        rotated_point = currentFrameToGroundRotMatrix_ * cv::Point3f(x_offset + texture_height,
                                                                     y_offset,
                                                                     z_offset);
        obj->position(rotated_point.x + transform.getOrigin().getX(),
                      rotated_point.y + transform.getOrigin().getY(),
                      rotated_point.z + transform.getOrigin().getZ());
        obj->textureCoord(1.0f, 0.0f);
        obj->normal(0.0f, 0.0f, 1.0f);

        // top right
        rotated_point = currentFrameToGroundRotMatrix_ * cv::Point3f(x_offset + texture_height,
                                                                     y_offset + texture_width,
                                                                     z_offset);
        obj->position(rotated_point.x + transform.getOrigin().getX(),
                      rotated_point.y + transform.getOrigin().getY(),
                      rotated_point.z + transform.getOrigin().getZ());
        obj->textureCoord(1.0f, 1.0f);
        obj->normal(0.0f, 0.0f, 1.0f);


        obj->end();


        MapObject object;
        object.object = obj;
        object.texture = texture;
        object.material = material;
        objects_.push_back(object);
    }



    // TODO(gareth): We are technically ignoring the orientation from the
    void CameraToGround_Display::transformGroundTexture() {

        tf::StampedTransform transform;
        try { listener_.lookupTransform(fixed_frame_.toUtf8().constData(), ORIGIN_FRAME, ros::Time(0), transform); }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
            return;
        }

        tf::Matrix3x3 camera_rot_matrix(transform.getRotation());
        cv::Matx33f camera_to_ground_rot_matrix( 3, 3, CV_32F );
        for (int i = 0 ; i < 3 ; i++) {
            currentFrameToGroundRotMatrix_(i,0) = (float)camera_rot_matrix.getRow(i).getX();
            currentFrameToGroundRotMatrix_(i,1) = (float)camera_rot_matrix.getRow(i).getY();
            currentFrameToGroundRotMatrix_(i,2) = (float)camera_rot_matrix.getRow(i).getZ();
        }
    }



    void CameraToGround_Display::fixedFrameChanged() {
        transformGroundTexture();
    }



    void CameraToGround_Display::reset() {
        Display::reset();
//
//        if (tf_filter_)
//            tf_filter_->clear();
    }


    void CameraToGround_Display::updateTopic()
    {
        unsubscribe();
        reset();
        subscribe();
        //context_->queueRender();
    }


    void CameraToGround_Display::fillTransportOptionList(EnumProperty* property)
    {
        property->clearOptions();
        std::vector<std::string> choices;
        choices.push_back("raw");

        // Loop over all current ROS topic names
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);
        ros::master::V_TopicInfo::iterator it = topics.begin();
        ros::master::V_TopicInfo::iterator end = topics.end();
        for (; it != end; ++it)
        {
            // If the beginning of this topic name is the same as topic_,
            // and the whole string is not the same,
            // and the next character is /
            // and there are no further slashes from there to the end,
            // then consider this a possible transport topic.
            const ros::master::TopicInfo& ti = *it;
            const std::string& topic_name = ti.name;
            const std::string& topic = camera_name_->getStdString();

            if (topic_name.find(topic) == 0 && topic_name != topic && topic_name[topic.size()] == '/'
                && topic_name.find('/', topic.size() + 1) == std::string::npos)
            {
                std::string transport_type = topic_name.substr(topic.size() + 1);

                // If the transport type string found above is in the set of
                // supported transport type plugins, add it to the list.
                if (transport_plugin_types_.find(transport_type) != transport_plugin_types_.end())
                {
                    choices.push_back(transport_type);
                }
            }
        }

        for (size_t i = 0; i < choices.size(); i++)
        {
            property->addOptionStd(choices[i]);
        }
    }




    void CameraToGround_Display::scanForTransportSubscriberPlugins()
    {
        pluginlib::ClassLoader<image_transport::SubscriberPlugin> sub_loader("image_transport",
                                                                             "image_transport::SubscriberPlugin");

        BOOST_FOREACH( const std::string& lookup_name, sub_loader.getDeclaredClasses() ) {
            // lookup_name is formatted as "pkg/transport_sub", for instance
            // "image_transport/compressed_sub" for the "compressed"
            // transport.  This code removes the "_sub" from the tail and
            // everything up to and including the "/" from the head, leaving
            // "compressed" (for example) in transport_name.
            std::string transport_name = boost::erase_last_copy(lookup_name, "_sub");
            transport_name = transport_name.substr(lookup_name.find('/') + 1);

            // If the plugin loads without throwing an exception, add its
            // transport name to the list of valid plugins, otherwise ignore
            // it.
            try {
                boost::shared_ptr<image_transport::SubscriberPlugin> sub = sub_loader.createInstance(lookup_name);
                transport_plugin_types_.insert(transport_name);
            }
            catch (const pluginlib::LibraryLoadException& e) { }
            catch (const pluginlib::CreateClassException& e) { }
        }
    }

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz::CameraToGround_Display, rviz::Display)
