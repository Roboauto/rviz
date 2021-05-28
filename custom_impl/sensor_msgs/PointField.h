#pragma once

#include <std_msgs/Header.h>

#include <stddef.h>

namespace sensor_msgs {
    struct PointField {
        static const uint8_t INT8 = 1;
        static const uint8_t UINT8 = 2;
        static const uint8_t INT16 = 3;
        static const uint8_t UINT16 = 4;
        static const uint8_t INT32 = 5;
        static const uint8_t UINT32 = 6;
        static const uint8_t FLOAT32 = 7;
        static const uint8_t FLOAT64 = 8;

        std::string name;
        unsigned int offset;
        unsigned char datatype;
        unsigned int count;

    };
}