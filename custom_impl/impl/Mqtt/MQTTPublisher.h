//
// Created by Miroslav Krajicek on 8/14/18.
//

#pragma once

#include "MQTTBase.h"

#include <RoboCore/MQTT/MQTTBase.h>
#include <RoboCore/Worker/Void.h>
#include <RoboCore/Logging/Logger.h>

#include <mosquittopp.h>
#include <msgpack/v3/pack_decl.hpp>
#include <msgpack/v3/sbuffer_decl.hpp>

#include <sstream>

namespace RoboCore::MQTT {

    template <class MessageType>
    class MQTTPublisher : public Worker::Void, protected mosqpp::mosquittopp {

    public:
        MQTTPublisher(const std::string &clientId, const MQTTServerSettings &server, const std::string &topic)
        : mosquittopp(clientId.c_str())
        , _clientId(clientId)
        , _serverSettings(server)
        , _topic(topic)
        {
            connect_async(server.host.c_str(),server.port);
            loop_start();
        }

        void publish(const std::shared_ptr<const MessageType> &data)
        {
            msgpack::v2::sbuffer ss;
            msgpack::pack(ss,*data.get());
            mosqpp::mosquittopp::publish(nullptr, _topic.c_str(), ss.size(), ss.data(), _serverSettings.QoS);
        }

    private:
        void on_connect(int rc) override
        {
            const std::string message = "MQTT client: " + _clientId + " ";

            switch (rc) {
                case 0:
                    ROBO_INFO(message + "connection successful");
                    break;
                case 1:
                    ROBO_ERROR(message + "connection refused (unacceptable protocol version)");
                    break;
                case 2:
                    ROBO_ERROR(message + "connection refused (identifier rejected)");
                    break;
                case 3:
                    ROBO_ERROR(message + "connection refused (broker unavailable)");
                    break;
                default:
                    ROBO_ERROR(message + "unknown return code");
                    break;
            }
        }

        void on_disconnect(int rc) override
        {
            const std::string message = "MQTT client: " + _clientId + " ";

            switch (rc) {
                case 0:
                    ROBO_INFO(message + "disconnected");
                    break;
                default:
                    ROBO_ERROR(message + "unexpected disconnect");
                    break;
            }
        }

        std::string _clientId;
        MQTTServerSettings _serverSettings;
        std::string _topic;
    };

}
