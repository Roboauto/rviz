#pragma once

#include <string>
#include <vector>
#include <sstream>
#include <ros/topic.h>
#include <impl/topics.h>


namespace ros { namespace names {
    inline bool validate(const std::string& base, const std::string& topic) {
        return true;
    }

    inline std::string parentNamespace(const std::string&) {

    }
}}

namespace ros { namespace master {

    inline void getTopics(V_TopicInfo &top) {
        top = impl::getTopics();
    }

    inline bool check() {
        return true; // TODO: impl!
    }

    inline std::string getURI() {
        return "TMP";
    }
}}