#pragma once

#include "MQTTBase.h"

#include <string>
#include <functional>

#include <mosquittopp.h>
#include <msgpack.hpp>
#include <iostream>
#include <thread>
#include <utility>

namespace MQTT {

  static const int k_mqtt_port  = 1883; // port which is reserved for mqtt communication

    template<class MessageType>
    class MQTTSubscriber : public mosqpp::mosquittopp {

    public:
        MQTTSubscriber(const std::string& clientId,
                       const MQTTServerSettings& server,
                       const std::string& topic,
                       std::function<void(std::shared_ptr<MessageType>&)> function = {})
                       : mosquittopp(clientId.c_str())
                       , _clientId(clientId)
                       , _serverSettings(server)
                       , _topic(topic)
                       , _receive_callback(function)
        {
            connect_async(_serverSettings.host.c_str(), _serverSettings.port);
            loop_start();
        }

        ~MQTTSubscriber() override = default;

        void setCallback(std::function<void(std::shared_ptr<MessageType>&)> function) {
            _receive_callback = function;
        }

        void unsubscribe() {
            mosquittopp::unsubscribe(nullptr, _topic.c_str());
        }

        /**
         * Subscriber will subscribe to the new topic
         * @param topic Topic to which to subscribe
         */
        void subscribe(const std::string& topic) {
            unsubscribe();
            _topic = topic;
            subscribe();
        }

        void setErrorHandler(std::function< bool(const std::string & )> error_handler){
            _error_handler = error_handler;
        }

        void resetServerSettings(const MQTTServerSettings& server ){
          unsubscribe();
          disconnect();
          _serverSettings = server;

          connect_async(_serverSettings.host.c_str(), _serverSettings.port);
          subscribe();
        }


    private:
        void on_connect(int rc) override {
            mosquittopp::subscribe(nullptr, _topic.c_str());
        }

        void on_message(const struct mosquitto_message* message) override {
            msgpack::object_handle result;
            try {
                msgpack::unpack(result, static_cast<const char *>(message->payload), message->payloadlen);
                msgpack::object obj = result.get();

                std::shared_ptr<MessageType> msgPtr(new MessageType());
                obj.convert(*msgPtr.get());
                _receive_callback(msgPtr);
            }
            catch ( msgpack::type_error &){
                if (_error_handler && _error_handler("Message has wrong type") == 1) {
                    //error handler wants to supress the error
                    return;
                }

                throw std::bad_cast(); //was not able to cast message
            }
        }

        /**
         * Resubscriber to the the previously subsrcibed topic
         */
        void subscribe() {
          mosquittopp::subscribe(nullptr, _topic.c_str());
        }

        std::string _clientId;
        MQTTServerSettings _serverSettings;
        std::string _topic;
        std::function<void(std::shared_ptr<MessageType>&)> _receive_callback;
        std::function<bool(const std::string &)> _error_handler;
    };
}
