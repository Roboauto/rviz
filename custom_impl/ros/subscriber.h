#pragma once

#include <vector>

#include "subscribe_options.h"

#include <impl/subscriber.h>

namespace ros {
    class Subscriber {
    public:
        template<class T>
        Subscriber(const std::string topic, unsigned qSize, std::function<void(const boost::shared_ptr<const T> &)> f)
                : sub_{impl::Subscriber::create(topic, qSize, f)} {
        }

        Subscriber() {

        }

        Subscriber(const Subscriber &rhs) = default;

        ~Subscriber() = default;

        void shutdown() {
            if (sub_) {
                sub_.reset();
            }
        }

    private:
        std::shared_ptr<impl::Subscriber::ISubscriber> sub_;
    };

}