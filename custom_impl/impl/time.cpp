#include "time.h"

static ros::Time simTime_{};

ros::Time impl::getSimTime() {
    return simTime_;
}
void impl::setSimTime(const ros::Time&t) {
    simTime_ = t;
}