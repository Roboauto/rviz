#pragma once

#include "image_encodings.h"

#include <std_msgs/Header.h>
#include <ros/message_traits.h>

namespace sensor_msgs {
    struct Image {
        using ConstPtr = boost::shared_ptr<const Image>;

        std_msgs::Header header;
        unsigned int height;
        unsigned int width;

        std::string encoding;
        unsigned char is_bigendian;
        unsigned int step;
        std::vector<uint8_t> data;
    };
};
namespace ros { namespace message_traits { // TODO: remove when used with our impl!

    template<>
    inline const char* datatype<sensor_msgs::Image>() {
        return "sensor_msgs/Image";
    }

} }