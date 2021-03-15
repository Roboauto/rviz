#pragma once

#include <visualization_msgs/MarkerArray.h>
#include <MQTTVisualizationMessages2/MarkerMsg.h>

#include <impl/conversions.h>

namespace impl { namespace conversions {
        template<>
        struct MsgAlternative<visualization_msgs::MarkerArray> {
            using type = MQTTVisualizationMessages2::MarkerArrayMsg;
        };

        template<>
        struct ROSConvertor<MQTTVisualizationMessages2::MarkerArrayMsg> {
            static inline visualization_msgs::MarkerArray convert(const MQTTVisualizationMessages2::MarkerArrayMsg& t) {
                visualization_msgs::MarkerArray ret;
                for(const auto marker : t.markers) {
                    ret.markers.emplace_back(toRos(marker));
                }
                return ret;
            }
        };
}}