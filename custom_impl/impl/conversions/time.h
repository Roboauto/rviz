#pragma once

#include <ros/time.h>
#include <MQTTVisualizationMessages2/Time.h>

#include <impl/conversions.h>

namespace impl { namespace conversions {
    template<>
    struct MsgAlternative<ros::Time> {
        using type = MQTTVisualizationMessages2::Time;
    };

    template<>
    struct ROSConvertor<MQTTVisualizationMessages2::Time> {
        static inline ros::Time convert(const MQTTVisualizationMessages2::Time& t) {
            ros::Time rosTime;
            rosTime.sec = t.sec;
            rosTime.nsec = t.nsec;
            return rosTime;
        }
    };

}}