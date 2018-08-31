/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <ros/time.h>

#include <tf/transform_listener.h>

#include "rviz/default_plugin/point_cloud_common.h"
#include "rviz/display_context.h"
#include "rviz/frame_manager.h"
#include "rviz/ogre_helpers/point_cloud.h"
#include "rviz/properties/int_property.h"

#include "point_cloud_display.h"

namespace rviz
{

PointCloudDisplay::PointCloudDisplay()
  : point_cloud_common_( new PointCloudCommon( this )),
    serverSettings_("127.0.0.1", 1883, MQTT::QOS::AT_LEAST_ONCE),
    _subscriber("rviz_pcl", serverSettings_, "", std::bind( &PointCloudDisplay::incomingMqttMessage, this, std::placeholders::_1))
{
  queue_size_property_ = new IntProperty( "Queue Size", 10,
                                          "Advanced: set the size of the incoming PointCloud message queue. "
                                          " Increasing this is useful if your incoming TF data is delayed significantly "
                                          "from your PointCloud data, but it can greatly increase memory usage if the messages are big.",
                                          this, SLOT( updateQueueSize() ));

  // PointCloudCommon sets up a callback queue with a thread for each
  // instance.  Use that for processing incoming messages.
  update_nh_.setCallbackQueue( point_cloud_common_->getCallbackQueue() );
}

PointCloudDisplay::~PointCloudDisplay()
{
  delete point_cloud_common_;
}

void PointCloudDisplay::onInitialize()
{
  MFDClass::onInitialize();
  point_cloud_common_->initialize( context_, scene_node_ );
}

void PointCloudDisplay::updateQueueSize()
{
  tf_filter_->setQueueSize( (uint32_t) queue_size_property_->getInt() );
}

void PointCloudDisplay::processMessage( const sensor_msgs::PointCloudConstPtr& cloud )
{
  point_cloud_common_->addMessage( cloud );
}

void PointCloudDisplay::update( float wall_dt, float ros_dt )
{
  point_cloud_common_->update( wall_dt, ros_dt );
}

void PointCloudDisplay::reset()
{
  MFDClass::reset();
  point_cloud_common_->reset();
}

void PointCloudDisplay::incomingMqttMessage( std::shared_ptr<RoboCore::PointCloudMsg> & message_ptr){
  incomingMqttMessage_(*message_ptr);
}

void PointCloudDisplay::incomingMqttMessage_(const RoboCore::PointCloudMsg & message) {
  sensor_msgs::PointCloud pcl;

  pcl.header.frame_id = message.frame_id;
  pcl.header.stamp = ros::Time(message.time_stamp);

  for(auto const &pt : message.points) {
    geometry_msgs::Point32 p;
    p.x = pt.x;
    p.y= pt.y;
    p.z = pt.z;
    pcl.points.push_back(p);
  }

  for(auto const &channel : message.channels) {
    sensor_msgs::ChannelFloat32 ch;
    ch.name = channel.first;
    for(auto const &pt : channel.second) {
      ch.values.push_back(pt);
    }
    pcl.channels.push_back(ch);
  }

  point_cloud_common_->addMessage(sensor_msgs::PointCloud::Ptr(new sensor_msgs::PointCloud(pcl)));
}

void PointCloudDisplay::subscribe()
{
  MessageFilterDisplay<sensor_msgs::PointCloud>::subscribe();
  _subscriber.subscribe(topic_property_->getValue().toString().toStdString());
}

void PointCloudDisplay::unsubscribe()
{
  MessageFilterDisplay<sensor_msgs::PointCloud>::unsubscribe();
  _subscriber.unsubscribe();
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::PointCloudDisplay, rviz::Display )
