/*
 * CameraToGround_Display.h
 */

#ifndef CAMERA_TO_GROUND_DISPLAY_H_
#define CAMERA_TO_GROUND_DISPLAY_H_

// NOTE: workaround for issue: https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <ros/time.h>
#include <rviz/display.h>
#include <sensor_msgs/NavSatFix.h>

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#endif  //  Q_MOC_RUN

#include <QObject>
#include <QFuture>
#include <QByteArray>
#include <QFile>

#include <memory>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <tf/transform_listener.h>
#include "tools/projection.h"

namespace Ogre {
class ManualObject;
}

namespace rviz {

class FloatProperty;
class IntProperty;
//class Property;
class RosTopicProperty;
//class StringProperty;
class TfFrameProperty;
class EnumProperty;

/**
 * @class CameraToGround_Display
 * @brief Displays camera to ground texture projection
 */
class CameraToGround_Display : public Display {
    Q_OBJECT

public:

    CameraToGround_Display();
    ~CameraToGround_Display() override;

    // Overrides from Display
    void onInitialize() override;
    void fixedFrameChanged() override;
    void reset() override;
    void update(float, float) override;

protected Q_SLOTS:

    void updateAlpha();
    void updateTextureLength();
    void updateTextureWidth();
    void updatePixelPerMeter();
    void updateCameraName();
    void updateCameraFrame();
    void updateOriginFrame();
    void fillTransportOptionList(EnumProperty* property);
    void updateTopic();

protected:

    // overrides from Display
    void onEnable() override;
    void onDisable() override;

    virtual void subscribe();
    virtual void unsubscribe();

    void assembleScene();

    void clear();

    void clearGeometry();

    void transformGroundTexture();

    void onNewImage(const sensor_msgs::ImageConstPtr& msg);

    void onNewCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg);

    void scanForTransportSubscriberPlugins();

    unsigned int map_id_;
    unsigned int scene_id_;

    image_transport::ImageTransport *it_;

    /// Instance of a tile w/ associated ogre data
    struct MapObject {
        Ogre::ManualObject *object;
        Ogre::TexturePtr texture;
        Ogre::MaterialPtr material;
    };

    std::vector<MapObject> objects_;

    boost::shared_ptr<image_transport::SubscriberFilter> image_sub_;
    ros::Subscriber camera_info_sub_;

    cv::Mat camera_image_;
    sensor_msgs::CameraInfo camera_info_;
    Projection *projectionTool_;
    tf::TransformListener listener_;
    cv::Matx33f currentFrameToGroundRotMatrix_;
    int messages_received_;

    //  properties
    IntProperty *length_in_meters_;
    IntProperty *width_in_meters_;
    IntProperty *pixels_per_meter_;
    FloatProperty *alpha_property_;
    RosTopicProperty *camera_name_;
    EnumProperty * transport_property_;
    TfFrameProperty *frame_property_;
    TfFrameProperty *origin_frame_property_;
    BoolProperty* unreliable_property_;

    std::set<std::string> transport_plugin_types_;

	
    // properties consts
    std::string ORIGIN_FRAME = "ground";

    const float TEXTURE_ALPHA_MAX_ = 1.0f;
    const float TEXTURE_ALPHA_MIN_ = 0.0f;
    const float TEXTURE_ALPHA_INIT_ = 0.7f;

    const int TEXTURE_LENGTH_MAX_ = 100;
    const int TEXTURE_LENGTH_MIN_ = 10;
    const int TEXTURE_LENGTH_INIT_ = 50;

    const int TEXTURE_WIDTH_MAX_ = 60;
    const int TEXTURE_WIDTH_MIN_ = 2;
    const int TEXTURE_WIDTH_INIT_ = 10;

    const int TEXTURE_PIXEL_PER_METER_MAX_ = 100;
    const int TEXTURE_PIXEL_PER_METER_MIN_ = 2;
    const int TEXTURE_PIXEL_PER_METER_INIT_ = 20;


    float alpha_;
    bool draw_under_;
    int texture_length_;
    int texture_width_;
    int texture_pixel_per_meter_;


};

} // namespace rviz

#endif
