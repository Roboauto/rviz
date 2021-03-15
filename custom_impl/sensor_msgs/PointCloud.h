#pragma once

#include <std_msgs/Header.h>

#include "geometry_msgs/Point.h"
#include "ChannelFloat32.h"

namespace sensor_msgs {
    struct PointCLoud2 {
        std_msgs::Header header;

        std::vector<geometry_msgs::Point> points;
        std::vector<ChannelFloat32> channels;
    };
};