#pragma once

#include "Vector3.h"
#include <std_msgs/Header.h>

namespace geometry_msgs {
    struct Vector3Stamped {
        std_msgs::Header header;
        Vector3 vector;
    };
}