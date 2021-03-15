#pragma once

#include "Pose.h"

#include <array>

namespace geometry_msgs {
    struct PoseWithCovariance {
        using _covariance_type = std::array<double, 36>;
        Pose pose;
        _covariance_type covariance;
    };
}