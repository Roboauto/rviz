#pragma once

#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>

#include <vector>

namespace nav_msgs {
    struct Path {
        using ConstPtr = boost::shared_ptr<const Path>;

        std_msgs::Header header{};
        std::vector<geometry_msgs::PoseStamped> poses{};
    };
}


#include <impl/conversions/path.h> // I dont want to modify ros files, so this is only solution i found