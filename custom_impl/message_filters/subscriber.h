#pragma once

#include <boost/thread/mutex.hpp>
#include <ros/node_handle.h>
#include <ros/message_event.h>

namespace message_filters {

    class SubscriberBase {
    public:
        virtual ~SubscriberBase() {}

        /**
         * \brief Subscribe to a topic.
         *
         * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
         *
         * \param nh The ros::NodeHandle to use to subscribe.
         * \param topic The topic to subscribe to.
         * \param queue_size The subscription queue size
         * \param transport_hints The transport hints to pass along
         * \param callback_queue The callback queue to pass along
         */
        virtual void subscribe(ros::NodeHandle &nh, const std::string &topic, uint32_t queue_size) = 0;

        /**
         * \brief Re-subscribe to a topic.  Only works if this subscriber has previously been subscribed to a topic.
         */
        virtual void subscribe() = 0;

        /**
         * \brief Force immediate unsubscription of this subscriber from its topic
         */
        virtual void unsubscribe() = 0;
    };

    typedef boost::shared_ptr<SubscriberBase> SubscriberBasePtr;

/**
 * \brief ROS subscription filter.
 *
 * This class acts as a highest-level filter, simply passing messages from a ROS subscription through to the
 * filters which have connected to it.
 *
 * When this object is destroyed it will unsubscribe from the ROS subscription.
 *
 * The Subscriber object is templated on the type of message being subscribed to.
 *
 * \section connections CONNECTIONS
 *
 * Subscriber has no input connection.
 *
 * The output connection for the Subscriber object is the same signature as for roscpp subscription callbacks, ie.
\verbatim
void callback(const boost::shared_ptr<M const>&);
\endverbatim
 */
    template<class M>
    class Subscriber : public SubscriberBase {

    public:
        typedef boost::shared_ptr<M const> MConstPtr;
        using EventType = ros::MessageEvent<const M>;

        class Connection {
        public:
            Connection() = default;
            Connection(Subscriber<M>* sub) : sub_(sub) {

            }

            void disconnect() {
                if(sub_) {
                    sub_->clearSubscribeFunction();
                    sub_ = nullptr;
                }
            }
        protected:
            Subscriber<M>* sub_{};
        };

        Subscriber(ros::NodeHandle &nh, const std::string &topic,
                   uint32_t queue_size ) : nh_{nh} {
            subscribe(nh, topic, queue_size/*, transport_hints, callback_queue*/);
        }

        /**
         * \brief Empty constructor, use subscribe() to subscribe to a topic
         */
        Subscriber() {
        }

        ~Subscriber() {
            unsubscribe();
        }

        /**
         * \brief Subscribe to a topic.
         *
         * If this Subscriber is already subscribed to a topic, this function will first unsubscribe.
         *
         * \param nh The ros::NodeHandle to use to subscribe.
         * \param topic The topic to subscribe to.
         * \param queue_size The subscription queue size
         * \param transport_hints The transport hints to pass along
         * \param callback_queue The callback queue to pass along
         */
        void subscribe(ros::NodeHandle &nh, const std::string &topic,
                       uint32_t queue_size) override {
            topic_ = topic;
            qSize_ = queue_size;
            subscribe();
        }

        /**
         * \brief Re-subscribe to a topic.  Only works if this subscriber has previously been subscribed to a topic.
         */
        void subscribe() override {
            unsubscribe();

            if (!topic_.empty()) {
                sub_ = nh_.subscribe(topic_, qSize_, &Subscriber<M>::processMessage, this);
            }
        }

        /**
         * \brief Force immediate unsubscription of this subscriber from its topic
         */
        void unsubscribe() override {
            sub_.shutdown();
        }

        Connection registerCallback(std::function<void(const boost::shared_ptr<const M>&)> fun) {
            assert(!function_);

            function_ = fun;
            return Connection(this);
        }

        void clearSubscribeFunction() {
            function_ = {};
        }
    private:
        void processMessage(const boost::shared_ptr<const M> &x) {
            if(function_) {
                function_(x);
            }
        }

        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        std::string topic_{};
        uint32_t qSize_{};
        std::function<void(const boost::shared_ptr<const M>&)> function_{};
    };

}
