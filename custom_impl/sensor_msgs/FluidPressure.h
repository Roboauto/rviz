#pragma once

#include <std_msgs/Header.h>

namespace sensor_msgs {
    struct FluidPreassure {
        using ConstPtr = std::shared_ptr<const Image>;
        double fluid_pressure;
        double variance;
    };
};