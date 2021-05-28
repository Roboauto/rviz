#pragma once

#include <std_msgs/Header.h>

#include "PointField.h"

namespace sensor_msgs {
    struct PointCloud2 {

      using ConstPtr = boost::shared_ptr<const PointCloud2>;

      std_msgs::Header header;

        uint32_t height;
        uint32_t width;

        std::vector<PointField> fields;

        bool is_bigendian;
        uint32_t point_step;
        uint32_t row_step;
        std::vector<uint8_t> data;

        bool is_dense;
    };

    using PointCloud2ConstPtr = boost::shared_ptr<const PointCloud2>;
    using PointCloud2Ptr = boost::shared_ptr<PointCloud2>;
};