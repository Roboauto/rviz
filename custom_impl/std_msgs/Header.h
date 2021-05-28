#pragma once

#include <ros/time.h>

namespace std_msgs {
    struct Header {
        unsigned int seq{};
        ros::Time stamp{};
        std::string frame_id {};
    };
}
