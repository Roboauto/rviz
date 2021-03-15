#pragma once

#include <MQTTVisualizationMessages2/Path.h>
#include <nav_msgs/Path.h>

#include "time.h"

#include <impl/conversions.h>


namespace impl { namespace conversions {
    template<>
    struct MsgAlternative<nav_msgs::Path> {
        using type = MQTTVisualizationMessages2::Path;
    };

    template<>
    struct ROSConvertor<MQTTVisualizationMessages2::Path> {
        static inline nav_msgs::Path convert(const MQTTVisualizationMessages2::Path& t) {
            nav_msgs::Path ret;
            ret.header.frame_id = t.frame;
            ret.header.stamp = toRos(t.time);
            for(auto p: t.poses) {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = p.point.x;
                pose.pose.position.y = p.point.y;
                pose.pose.position.z = p.point.z;

                ret.poses.emplace_back(pose);
            }

            return ret;
        }
    };
}}