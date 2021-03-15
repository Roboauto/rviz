#pragma once

#include "callback_queue.h"

#include "master.h"

#include <boost/shared_ptr.hpp>
#include <impl/init.h>


namespace ros {
    CallbackQueue *getGlobalCallbackQueue();

    enum class init_options {
        NoSigintHandler = 1 << 0,
        AnonymousName = 1 << 1,
        NoRosout = 1 << 2,
    };

    inline void init(int argc, char **argv, std::string names, init_options) {
        impl::init(ros::master::getURI());
    }
}