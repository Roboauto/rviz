#pragma once

#include <std_msgs/Header.h>
#include "Pose.h"

namespace geometry_msgs {
    struct PoseStamped {
        std_msgs::Header header;
        Pose pose;
    };
}