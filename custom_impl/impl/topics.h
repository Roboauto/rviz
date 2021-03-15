#pragma once

#include <ros/topic.h>

#include <impl/init.h>

namespace impl {

    inline std::vector<ros::master::TopicInfo> getTopics() {
        return initGetTopics();
    }
}