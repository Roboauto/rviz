#pragma once

#include <std_msgs/Header.h>
#include "Transform.h"
#include <ros/message_traits.h>

namespace geometry_msgs {
    struct TransformStamped {
        std_msgs::Header header{};
        Transform transform{};
        std::string child_frame_id{};
    };
}

namespace ros {namespace message_traits {
        template<>
        struct IsMessage<geometry_msgs::TransformStamped> {
            static const bool value = true;

        };
}}

#include <impl/conversions/tf.h>