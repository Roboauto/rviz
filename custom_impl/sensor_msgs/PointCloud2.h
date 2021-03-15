#pragma once

#include <std_msgs/Header.h>

#include "PointField.h"

namespace sensor_msgs {
    struct PointCLoud2 {
        std_msgs::Header header;

        unsigned int height;
        unsigned int width;

        std::vector<PointField> fields;

        bool is_bigendian;
        unsigned int point_step;
        unsigned int row_step;
        std::vector<unsigned char> data;

        bool is_dense;

        double fluid_pressure;
        double variance;
    };
};