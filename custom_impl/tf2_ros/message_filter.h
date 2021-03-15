#pragma once

#include <tf2_ros/buffer.h>

#include <message_filters/subscriber.h>
#include <image_transport/subscriber_filter.h>

namespace tf2_ros {
    namespace filter_failure_reasons {
        enum FilterFailureReason {
            /// The message buffer overflowed, and this message was pushed off the back of the queue, but the reason it was unable to be transformed is unknown.
            Unknown,
            /// The timestamp on the message is more than the cache length earlier than the newest data in the transform cache
            OutTheBack,
            /// The frame_id on the message is empty
            EmptyFrameID,
        };
    }
    typedef filter_failure_reasons::FilterFailureReason FilterFailureReason;

    template<class T>
    class Signal {
    public:
        using Ptr = boost::shared_ptr<const T>;

        void registerCallback(std::function<void(const Ptr &)> fun) {
            callbacks_.template emplace_back(fun);
        }

        void operator()(const Ptr &ptr) {
            for (auto x : callbacks_) {
                x(ptr);
            }
        }

    protected:
        std::vector<std::function<void(const Ptr &)>> callbacks_;
    };

    template<class T>
    class MessageFilter {
    protected:
        Signal<T> sig_;
        typename message_filters::Subscriber<T>::Connection connection_;
        Buffer &buffer_;
        std::string targetFrame_;
    public:
        MessageFilter(Buffer &buffer, const std::string &frame, int qSize, ros::NodeHandle &) : buffer_(buffer), targetFrame_{frame} {

        }

        MessageFilter(image_transport::SubscriberFilter &filter, Buffer &buffer, const std::string &frame, int qSize,
                      ros::NodeHandle &) : buffer_(buffer), targetFrame_{frame} {

        }

        void connectInput(message_filters::Subscriber<T> &sub) {
            connection_.disconnect();
            connection_ = sub.registerCallback(std::function{[this](const boost::shared_ptr<const T> &a) {
                sig_(a);
            }});
        }

        template<typename C>
        void registerCallback(C x) {
            sig_.registerCallback(std::function{[x](const boost::shared_ptr<const T> &a) {
                x(a);
            }});
        }


        template<typename C>
        void registerFailureCallback(C x) {
//    typename CallbackHelper1<M>::Ptr helper = signal_.addCallback(Callback(callback));
//    return Connection(boost::bind(&Signal::removeCallback, &signal_, helper));
        }

        void setQueueSize(int) {

        }

        void add(boost::shared_ptr<T> ptr) {
            boost::shared_ptr<T> ptr2 = boost::make_shared<T>();
            auto transform = buffer_.lookupTransform(targetFrame_, ptr2->header.frame_id, ptr2->header.stamp);
            tf2::doTransform(ptr, ptr2, transform);
            sig_(ptr2);
        }

        void setTargetFrame(const std::string &frameName) {
            targetFrame_ = frameName;
        }

        void clear() {

        }
    };
}