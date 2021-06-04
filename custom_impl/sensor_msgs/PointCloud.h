#pragma once

#include <std_msgs/Header.h>

#include "geometry_msgs/Point32.h"
#include "ChannelFloat.h"

namespace sensor_msgs {

    struct PointCloud {

        using ConstPtr = boost::shared_ptr<const PointCloud>;

        std_msgs::Header header;

        std::vector<geometry_msgs::Point32> points;
        std::vector<ChannelFloat> channels;
    };

    using PointCloudConstPtr = boost::shared_ptr<const PointCloud>;
};

#include <impl/conversions/pointCloud.h> // I dont want to modify ros files, so this is only solution i found