#pragma once

#include <string>

#include <ros/topic.h>

namespace impl {
    void init(const std::string& uri);

    std::vector<ros::master::TopicInfo> initGetTopics();
}