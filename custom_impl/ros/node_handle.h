#pragma once

#include <ros/time.h>
#include <ros/transport_hints.h>

#include "callback_queue_interface.h"
#include "subscriber.h"

#include <boost/bind.hpp>

//#include "Mqtt/MQTTBase.h"
//#include "subscribe_options.h"
#include "publisher.h"
#include "init.h"

namespace ros {

    class NodeHandle {
    protected:
        //MQTT::MQTTServerSettings mqttServerSettings_{"127.0.0.1", 1883, MQTT::QOS::AT_LEAST_ONCE};

        CallbackQueueInterface *callback_queue_{nullptr};

    public:
        NodeHandle() {
        }

        void setCallbackQueue(CallbackQueueInterface *queue) {
            callback_queue_ = queue;
        }

        CallbackQueueInterface *getCallbackQueue() const {
            return callback_queue_ ? callback_queue_ : getGlobalCallbackQueue();
        }

        template<class T>
        Publisher advertise(const std::string topic, int queue_size, bool latch=false) {
            return Publisher(topic, queue_size, latch);
        }

        template<class M, class T>
        Subscriber subscribe(const std::string &topic, uint32_t queue_size, void(T::*fp)(M), T *obj, const ros::TransportHints& transport_hints = ros::TransportHints()) {
            return Subscriber(topic, queue_size, std::function{[obj, fp](const M& x) {
                (obj->*fp)(x);
            }});
        }
    };

    using NodeHandlePtr = std::shared_ptr<NodeHandle>;
}