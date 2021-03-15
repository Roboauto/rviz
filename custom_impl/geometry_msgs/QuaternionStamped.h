#pragma once

#include "Quaternion.h"

namespace geometry_msgs {
    struct QuaternionStamped {
        std_msgs::Header header;
        Quaternion quaternion;
    };
}