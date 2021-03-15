#include "init.h"

#include <MQTTVisualizationMessages2/TopicMsg.h>
#include <mutex>

#include "time.h"
#include "subscriber.h"

namespace impl { namespace conversions {
    template<>
    struct MsgAlternative<std::vector<ros::master::TopicInfo>> {
        using type = MQTTVisualizationMessages2::Topics;
    };

    template<>
    struct ROSConvertor<MQTTVisualizationMessages2::Topics> {
        static inline std::vector<ros::master::TopicInfo> convert(const MQTTVisualizationMessages2::Topics &t) {
            std::vector<ros::master::TopicInfo> topics;
            for (const auto &topic: t.topics) {
                topics.emplace_back(topic.name, topic.type);
            }
            return topics;
        }
    };

}}

namespace impl {

    std::shared_ptr<Subscriber::ISubscriber> timeSubscriber_;
    std::shared_ptr<Subscriber::ISubscriber> topicsSubscibrer_;

    std::mutex topicMutex_;

    std::vector<ros::master::TopicInfo> topics_;

    void init(const std::string &uri) {
        timeSubscriber_ = Subscriber::create<ros::Time>("/time", 100, std::function{
                [](const boost::shared_ptr<const ros::Time> &time) {
                    setSimTime(*time);
                }});

        topicsSubscibrer_ = Subscriber::create<std::vector<ros::master::TopicInfo>>("/topics", 100, std::function{
                [](const boost::shared_ptr<const std::vector<ros::master::TopicInfo>> &data) {
                    std::lock_guard l(topicMutex_);
                    topics_ = *data;
                }});
    }

    std::vector<ros::master::TopicInfo> initGetTopics() {
        std::lock_guard l(topicMutex_);
        return topics_;
    }
}