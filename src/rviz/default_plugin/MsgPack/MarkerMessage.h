//
// Created by martin on 8/22/18.
//

#ifndef RVIZ_MARKERMESSAGE_H
#define RVIZ_MARKERMESSAGE_H


#include <msgpack.hpp>

#include <vector>
#include <string>
#include <cstdint>
#include <array>


struct MarkerMsg {
  uint32_t sequence_id;
  std::string frame_id;
  double time_stamp;

  std::string name_space;

  int32_t id;
  int32_t type;
  int32_t action;

  double position[3]; //position
  double orientation[4]; //quaternion
  double scale[3];
  float color[4];

  double lifetime;

  bool frame_locked;

  std::vector< std::array<double, 3> > points;
  std::vector< std::array<float, 4> >colors;

  std::string text;
  std::string mesh_resource;
  bool mesh_use_embedded_materials;

  MSGPACK_DEFINE(sequence_id, frame_id, time_stamp, name_space, id, type, action, position, orientation, scale, color,
    lifetime, frame_locked, points, colors, text, mesh_resource, mesh_use_embedded_materials);

};

struct MarkerArrayMsg {
  std::vector<MarkerMsg> markers;
  MSGPACK_DEFINE(markers);
};







#endif //RVIZ_MARKERMESSAGE_H
