#pragma once

#include "PoseWithCovariance.h"
#include <std_msgs/Header.h>

namespace geometry_msgs {
    struct PoseWithCovarianceStamped {
        std_msgs::Header header;
        PoseWithCovariance pose;
    };
}