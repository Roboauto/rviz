#pragma once

#include "Point.h"
#include <std_msgs/Header.h>

namespace geometry_msgs {
    struct PointStamped {
        std_msgs::Header header;
        Point point;
    };
}