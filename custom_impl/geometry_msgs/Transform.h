#pragma once

#include <std_msgs/Header.h>
#include "Vector3.h"
#include "Quaternion.h"

namespace geometry_msgs {
    struct Transform {
        Vector3 translation;
        Quaternion rotation;
    };
}