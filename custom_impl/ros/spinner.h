#pragma once

#include "callback_queue.h"
#include <boost/shared_ptr.hpp>

namespace ros {

/**
 * \brief AsyncSpinner is a spinner that does not conform to the abstract Spinner interface.  Instead,
 * it spins asynchronously when you call start(), and stops when either you call stop(), ros::shutdown()
 * is called, or its destructor is called
 *
 * AsyncSpinner is reference counted internally, so if you copy one it will continue spinning until all
 * copies have destructed (or stop() has been called on one of them)
 */
    class AsyncSpinner {
    public:
        AsyncSpinner(uint32_t thread_count) {
            throw std::runtime_error("Not implemented");
        }

        AsyncSpinner(uint32_t thread_count, CallbackQueue *queue) {
            throw std::runtime_error("Not implemented");
        }

        /**
         * \brief Start this spinner spinning asynchronously
         */
        void start() {
            throw std::runtime_error("Not implemented");
        }

        /**
         * \brief Stop this spinner from running
         */
        void stop() {
            throw std::runtime_error("Not implemented");
        }

    private:
        //AsyncSpinnerImplPtr impl_;
    };

}