#pragma once

#include <functional>

#include "Mqtt/MQTTSubscriber.h"
#include <MQTTVisualizationMessages2/Time.h>
#include <boost/make_shared.hpp>

#include "conversions.h"

namespace impl { namespace Subscriber {
        class ISubscriber {
        };

        template<class F, class T>
        class Subscriber : public ISubscriber {
        public:
            Subscriber(std::string id, const MQTT::MQTTServerSettings& server, std::string topic,
                       std::function<void(const boost::shared_ptr<const T>&)> callback): sub_{id, server, topic}, callback_{callback} {
                sub_.setCallback([this](auto time) {
                    callback_(boost::make_shared<T>(conversions::toRos(*time)));
                });
            }
        protected:
            MQTT::MQTTSubscriber<F> sub_;
            std::function<void(const boost::shared_ptr<const T>&)> callback_;
        };

        template<class T>
        std::shared_ptr<ISubscriber> create(const std::string& topic, int qSize, std::function<void(const boost::shared_ptr<const T>&)> f) {
            MQTT::MQTTServerSettings settings{"localhost", 1883};
            return std::make_shared<Subscriber<conversions::msgAlternative_t<T>, T>>("sub_" + topic, settings, topic, f);
        }

} }