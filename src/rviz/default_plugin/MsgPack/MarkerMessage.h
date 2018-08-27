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

#include <Types.h>


namespace RoboCore::Visualizer {
  enum MarkerType {
    ARROW = 0,
    CUBE,
    SPEHERE,
    CYLINDER,
    LINE_STRIP,
    LINE_LIST,
    CUBE_LIST,
    SPHERE_LIST,
    POINTS,
    TEXT_VIEW_FACING,
    MESH_RESOURCE,
    TRIANGLE_LIST
  };

  struct MarkerMsg {
    int sequence_id;
    std::string frame_id;
    double time_stamp;

    std::string name_space;

    int id;
    int type;
    int action;

    PoseXYZ position; //position
    Orientation orientation; //quaternion
    Scale scale;
    Color color;

    double lifetime;

    bool frame_locked;

    std::vector<PoseXYZ> points;
    std::vector<Color> colors;

    std::string text;
    std::string mesh_resource;
    bool mesh_use_embedded_materials;

    MSGPACK_DEFINE (sequence_id, frame_id, time_stamp, name_space, id, type, action, position, orientation, scale,
                    color, lifetime, frame_locked, points, colors, text, mesh_resource, mesh_use_embedded_materials)

  };

  struct MarkerArrayMsg {
    std::vector<MarkerMsg> markers;
    MSGPACK_DEFINE (markers)
  };
}

#endif //RVIZ_MARKERMESSAGE_H
