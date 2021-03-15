#pragma once

#include "image_transport.h"

namespace image_transport {
    class SubscriberFilter {
    public:
        void subscribe(ImageTransport&tr, const std::string &topic, int qSize, TransportHints hints) {

        }

        template<class T> //callback with const sensor_msgs::Image::ConstPtr& msg
        void registerCallback(T) {

        }
    };
}