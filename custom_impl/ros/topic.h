#pragma once

#include <string>
#include <vector>
#include <sstream>


namespace ros { namespace master {
    struct TopicInfo {
        TopicInfo() {
        }

        TopicInfo(const std::string &_name, const std::string &_datatype /*, const std::string& _md5sum*/)
                : name(_name), datatype(_datatype) {
        }

        std::string name;     ///< Name of the topic
        std::string datatype; ///< Datatype of the topic
    };

    typedef std::vector<TopicInfo> V_TopicInfo;
}}