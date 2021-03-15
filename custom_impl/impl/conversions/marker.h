#pragma once

#include <visualization_msgs/Marker.h>
#include <MQTTVisualizationMessages2/MarkerMsg.h>

#include "time.h"

namespace impl { namespace conversions {
    template<>
    struct MsgAlternative<visualization_msgs::Marker> {
        using type = MQTTVisualizationMessages2::MarkerMsg;
    };

    template<>
    struct ROSConvertor<MQTTVisualizationMessages2::MarkerAction> {
        static inline int convert(const MQTTVisualizationMessages2::MarkerAction &message) {
            switch(message) {
                case MQTTVisualizationMessages2::MarkerAction::Add:
                case MQTTVisualizationMessages2::MarkerAction::Modify:
                    return 0;
                case MQTTVisualizationMessages2::MarkerAction::Delete:
                    return 2;
                case MQTTVisualizationMessages2::MarkerAction::DelteAll:
                    return 3;
            }
        }
    };

    template<>
    struct ROSConvertor<MQTTVisualizationMessages2::MarkerType> {
        static inline int convert(const MQTTVisualizationMessages2::MarkerType &message) {
            return static_cast<int>(message);
        }
    };

    template<>
    struct ROSConvertor<MQTTVisualizationMessages2::MarkerMsg> {
        static inline visualization_msgs::Marker convert(const MQTTVisualizationMessages2::MarkerMsg &message) {
            visualization_msgs::Marker marker;

            marker.header.seq = message.sequence_id;
            marker.header.frame_id = message.frame;
            marker.header.stamp = toRos(message.time);

            marker.ns = message.name_space;
            marker.id = message.id;
            marker.type = toRos(message.type);
            marker.action = toRos(message.action);

            marker.pose.position.x = message.position.x;
            marker.pose.position.y = message.position.y;
            marker.pose.position.z = message.position.z;
            marker.pose.orientation.w = message.orientation.w;
            marker.pose.orientation.x = message.orientation.x;
            marker.pose.orientation.y = message.orientation.y;
            marker.pose.orientation.z = message.orientation.z;

            marker.scale.x = message.scale.x;
            marker.scale.y = message.scale.y;
            marker.scale.z = message.scale.z;

            marker.color.r = message.color.r;
            marker.color.g = message.color.g;
            marker.color.b = message.color.b;
            marker.color.a = message.color.a;

            marker.lifetime = ros::Duration(message.lifetime);
            marker.frame_locked = message.frame_locked;

            for (auto point : message.points) {
                geometry_msgs::Point pt;
                pt.x = point.x;
                pt.y = point.y;
                pt.z = point.z;
                marker.points.push_back(pt);
            }

            for (auto color : message.colors) {
                std_msgs::ColorRGBA next_color;
                next_color.r = color.r;
                next_color.g = color.g;
                next_color.b = color.b;
                next_color.a = color.a;
                marker.colors.push_back(next_color);
            }

            marker.text = message.text;
            marker.mesh_resource = message.mesh_resource;
            marker.mesh_use_embedded_materials = message.mesh_use_embedded_materials;
            return marker;
        }
    };
}}