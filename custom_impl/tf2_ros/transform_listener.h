#pragma once

#include <tf2_ros/buffer.h>
#include <ros/node_handle.h>
#include "message_filter.h"

#include <impl/subscriber.h>


namespace tf2_ros {
    class TransformListener {

    public:

        TransformListener(tf2_ros::Buffer &buffer, const ros::NodeHandle &nh, bool spin_thread = true) : buffer_{
                buffer} {
            sub_ = impl::Subscriber::create(std::string{"/tf"}, 100, std::function{
                    [this](const boost::shared_ptr<const geometry_msgs::TransformStamped> &data) {
                        processTFUpdate(*data);
                    }});
        }

        ~TransformListener() = default;

    protected:
        void processTFUpdate(const geometry_msgs::TransformStamped &tf) {
            buffer_.setTransform(tf, "RC");
        }

        tf2_ros::Buffer &buffer_;
        std::shared_ptr<impl::Subscriber::ISubscriber> sub_{};
    };
}