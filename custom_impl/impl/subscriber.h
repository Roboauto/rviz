#pragma once

#include <functional>

#include "Mqtt/MQTTSubscriber.h"
#include <MQTTVisualizationMessages2/Time.h>
#include <boost/make_shared.hpp>

#include "conversions.h"
#include "init.h"

namespace impl { namespace Subscriber {
    class ISubscriber
    {
    };

  template <class F, class T>
   class Subscriber : public ISubscriber
  {
  public:
    Subscriber(std::string id,
               const MQTT::MQTTServerSettings& server,
               std::string topic,
               std::function<void(const boost::shared_ptr<const T>&)> callback,
               std::function<void(const std::string& error)> errorCallback)
      : sub_{id, server, topic}, callback_{callback}, errorCallback_{errorCallback}
    {
      sub_.setCallback([this](auto time) { callback_(boost::make_shared<T>(conversions::toRos(*time))); });
      sub_.setErrorHandler([this](const std::string& error) {
        errorCallback_(error);
        return true;
      });
    }

  protected:
    MQTT::MQTTSubscriber<F> sub_;
    std::function<void(const boost::shared_ptr<const T>&)> callback_;
    std::function<void(const std::string& error)> errorCallback_;
  };

  template <class T>
  std::shared_ptr<ISubscriber>
  create(const std::string& topic, int qSize, std::function<void(const boost::shared_ptr<const T>&)> f, std::function<void(const std::string& error)> errorCallback)
  {
    MQTT::MQTTServerSettings settings{"localhost", 1883};
    auto topics = initGetTopics();
    for(const auto top: topics) {
      if(top.name == topic ) {
        if(top.datatype != conversions::datatype<conversions::msgAlternative_t<T>>()) {
          throw std::runtime_error("Topic type mismatch");
        }
        break;
      }
    }

    return std::make_shared<Subscriber<conversions::msgAlternative_t<T>, T>>("sub_" + topic, settings,
                                                                             topic, f, errorCallback);
  }

} }