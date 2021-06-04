#pragma once

#include <MQTTVisualizationMessages2/PointCloudMsg.h>
#include <nav_msgs/Path.h>

#include "time.h"

#include <impl/conversions.h>


namespace impl { namespace conversions {
template<>
struct MsgAlternative<sensor_msgs::PointCloud> {
  using type = MQTTVisualizationMessages2::PointCloudMsg;
};

template<>
struct ROSConvertor<MQTTVisualizationMessages2::PointCloudMsg> {
  static inline sensor_msgs::PointCloud convert(const MQTTVisualizationMessages2::PointCloudMsg& t) {
    sensor_msgs::PointCloud ret;
    ret.header.frame_id = t.frame;
    ret.header.stamp = toRos(t.time);
    ret.header.seq = t.sequence_id;

    ret.points.reserve(t.points.size());

    std::for_each(t.points.begin(), t.points.end(), [&ret](auto ptx) {
      ret.points.push_back({ptx.x, ptx.y, ptx.z});
    });

    std::for_each(t.channels.begin(), t.channels.end(), [&ret](auto data) {
      sensor_msgs::ChannelFloat channel{};
      channel.name = data.first;
      channel.values = data.second;
      ret.channels.emplace_back(std::move(channel));
    });
    return ret;
  }
};
}}