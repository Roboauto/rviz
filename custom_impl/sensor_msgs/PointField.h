#pragma once

#include <std_msgs/Header.h>

namespace sensor_msgs {
    struct PointField {
        uint8 INT8 = 1
        uint8 UINT8 = 2
        uint8 INT16 = 3
        uint8 UINT16 = 4
        uint8 INT32 = 5
        uint8 UINT32 = 6
        uint8 FLOAT32 = 7
        uint8 FLOAT64 = 8

        std::string name;
        unsigned int offset;
        unsigned char datatype;
        unsigned int count;

    };
};