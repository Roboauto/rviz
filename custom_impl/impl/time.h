#pragma once

#include <ros/time.h>

namespace impl {
    ros::Time getSimTime();
    void setSimTime(const ros::Time&);
}