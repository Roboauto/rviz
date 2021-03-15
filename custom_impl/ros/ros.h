#pragma once

#include "package.h"
#include "master.h"
#include "message_event.h"
#include "exception.h"
#include "assert.h"
#include "init.h"
#include "node_handle.h"


namespace ros {

    inline bool ok() {
        return true; // TODO: impl!
    }

    inline void spinOnce() {
        getGlobalCallbackQueue()->callAvailable(ros::WallDuration());
    }

    inline bool isInitialized() {
        return true;
    }
}