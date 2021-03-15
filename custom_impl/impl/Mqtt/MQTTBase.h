//
// Created by Miroslav Krajicek on 8/13/18.
//

#pragma once

//#include <msgpack.hpp>

#include <string>

namespace MQTT {
    static const uint16_t k_std_mqtt_port = 1883;
    static const uint16_t k_std_mqtt_ssl_port = 8883;

    enum QOS{
      AT_MOST_ONCE = 0,
      AT_LEAST_ONCE = 1,
      EXACTLY_ONCE = 2
    };

    class MQTTMessage {
    public:
        std::string message;
//        MSGPACK_DEFINE(message)
    };

    struct MQTTServerSettings {
        std::string host;
        int port = k_std_mqtt_port;
        int QoS = QOS::AT_LEAST_ONCE;
    };


}
