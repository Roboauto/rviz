#pragma once

#include "Vector3.h"

namespace geometry_msgs {
    struct Wrench {
        Vector3 force{};
        Vector3 torque{};
    };
}