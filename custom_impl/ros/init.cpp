#include "init.h"

#include <boost/thread.hpp>

namespace ros {
    std::shared_ptr<CallbackQueue> globalQueue{};

    boost::thread globalQueueThread = boost::thread{[] () { // TODO: beter impl!
        while (true) {
            getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
        }
    }};

    CallbackQueue* getGlobalCallbackQueue() {
        if(!globalQueue) {
            globalQueue= std::make_shared<CallbackQueue>();
        }
        //globalQueue->enable();
        return globalQueue.get();
    }

}
