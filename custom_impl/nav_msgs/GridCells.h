#pragma once

#include <boost/shared_ptr.hpp>
#include <std_msgs/Header.h>
#include <geometry_msgs/Point.h>
#include <vector>

namespace nav_msgs {
    struct GridCells {
        using ConstPtr = boost::shared_ptr<const GridCells>;
        std_msgs::Header header{};
        float cell_width{};
        float cell_height{};
        std::vector<geometry_msgs::Point> cells{};

    };
}

#include <impl/conversions/gridCells.h> // I dont want to modify ros files, so this is only solution i found