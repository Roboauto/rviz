#pragma once

#include <vector>

namespace sensor_msgs {
    struct ChannelFloat {
        std::string name;

        std::vector<float> value;
    };
};