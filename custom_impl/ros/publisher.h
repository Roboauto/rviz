#pragma once

#include <impl/publisher.h>


namespace ros {
    class Publisher {
    public:
        Publisher() {

        }

        Publisher(const std::string topic, int queue_size, bool latch=false) : topic_{topic}, qSize_{queue_size}, latch_{latch}  {

        }

        Publisher(const Publisher &rhs) = default;

        ~Publisher() = default;

        template<typename M>
        void publish(const boost::shared_ptr<M> &message) const {
            publish(*message);
        }

        template<typename M>
        void publish(const M &message) {
            if(!publisher_) {
                publisher_ = impl::Publisher::create<M>(topic_, qSize_, latch_);
            }

            publisher_->publish(message);
        }

        void shutdown() {
            if(publisher_) {
                publisher_.reset();
            }
        }

    private:
        std::string topic_{};
        int qSize_{};
        bool latch_{};

        std::shared_ptr<impl::Publisher::Publisher> publisher_;
    };

}