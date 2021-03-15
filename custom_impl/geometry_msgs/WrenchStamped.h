#pragma once

#include "Wrench.h"

namespace geometry_msgs {
    struct WrenchStamped {
        std_msgs::Header header;
        Wrench wrench;
    };
}