#pragma once

#include "Types.h"
#include <iostream>

#include <msgpack.hpp>

namespace RoboCore {

    typedef std::pair<std::string, std::vector<float>> Channel;

    struct PointCloudMsg {
        std::string frame_id;
        unsigned time_stamp;

        std::string name_space;

        std::vector< PoseXYZ > points;
        std::vector< Channel > channels;

        MSGPACK_DEFINE(frame_id, time_stamp, name_space, points, channels)
    };
}