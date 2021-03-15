#pragma once

#include <boost/signals2.hpp>

namespace ros {

    template<class T>
    struct MessageEvent {
        MessageEvent(boost::shared_ptr<const T> msg) : msg_{msg} {

        }

        const boost::shared_ptr<const T>& getConstMessage() const {
            return msg_;
        }

        const std::string& getPublisherName() const {
            return pub_;
        };

    private:
        std::string pub_;
        boost::shared_ptr<const T> msg_;
    };
}