#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <MQTTVisualizationMessages2/TF.h>

#include "time.h"
#include <impl/conversions.h>

namespace impl { namespace conversions {
    template<>
    struct MsgAlternative<geometry_msgs::TransformStamped> {
        using type = MQTTVisualizationMessages2::TF;
    };

    template<>
    struct ROSConvertor<MQTTVisualizationMessages2::TF> {
        static inline geometry_msgs::TransformStamped convert(const MQTTVisualizationMessages2::TF& t) {
            geometry_msgs::TransformStamped rosTF;

            rosTF.child_frame_id = t.to_frame;
            rosTF.header.stamp = toRos(t.time);
            rosTF.header.frame_id = t.from_frame;

            rosTF.transform.translation.x = t.vector.x;
            rosTF.transform.translation.y = t.vector.y;
            rosTF.transform.translation.z = t.vector.z;

            rosTF.transform.rotation.x = t.rotation.x;
            rosTF.transform.rotation.y = t.rotation.y;
            rosTF.transform.rotation.z = t.rotation.z;
            rosTF.transform.rotation.w = t.rotation.w;

            return rosTF;
        }
    };
}}