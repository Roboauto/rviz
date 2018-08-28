//
// Created by Miroslav Krajicek on 8/13/18.
//

#pragma once

#include "MQTTBase.h"

#include <string>
#include <functional>

#include <mosquittopp.h>
#include <msgpack.hpp>

namespace MQTT {

    template <class MessageType>
    class MQTTSubscriber : public mosqpp::mosquittopp {

    public:
      MQTTSubscriber(const std::string &clientId)
        : mosquittopp(clientId.c_str())
        , _clientId(clientId)
        , _topic() {
        loop_start();
      }

      void subscribe(const std::string &topic, MQTT::QOS qos) {

        mosquittopp::unsubscribe(nullptr, _topic.c_str());
        _topic = topic;
        mosquittopp::subscribe(nullptr, _topic.c_str(), qos);
      }

      void unsubscribe() {
        mosquittopp::unsubscribe(nullptr, _topic.c_str());
      }


      void setCallback(std::function<void(std::shared_ptr<MessageType> &)> function) {
        _receive_callback = function;
      }

    private:
      void on_connect(int rc) override {
        //mosquittopp::subscribe(nullptr, _topic.c_str());
      }

      void on_message(const struct mosquitto_message *message) override {
        msgpack::object_handle result;
        msgpack::unpack(result, static_cast<const char *>(message->payload), message->payloadlen);
        msgpack::object obj = result.get();

        std::shared_ptr<MessageType> msgPtr(new MessageType());
        obj.convert(*msgPtr.get());

        _receive_callback(msgPtr);
      }

/*
        void on_subscribe(int mid, int qos_count , const int *granted_qos) override {}
        void on_unsubscribe(int mid) override {}

        //TODO check if needed
        void on_log(int level, const char *str) override {}
        void on_error() override {}
*/
      std::string _clientId;
      MQTTServerSettings _serverSettings;
      std::string _topic;
      std::function<void(std::shared_ptr<MessageType> &)> _receive_callback;
    };
}
