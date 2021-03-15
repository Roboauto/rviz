#pragma once

#include <geometry_msgs/Quaternion.h>
#include "Marker.h"

namespace visualization_msgs {
    struct InteractiveMarkerControl {
        std::string name;

        geometry_msgs::Quaternion orientation;

        static const unsigned char INHERIT = 0;
        static const unsigned char FIXED = 1;
        static const unsigned char VIEW_FACING = 2;

        static const unsigned char orientation_mode;

        static const unsigned char NONE = 0;
        static const unsigned char MENU = 1;
        static const unsigned char BUTTON = 2;
        static const unsigned char MOVE_AXIS = 3;
        static const unsigned char MOVE_PLANE = 4;
        static const unsigned char ROTATE_AXIS = 5;
        static const unsigned char MOVE_ROTATE = 6;
        static const unsigned char MOVE_3D = 7;
        static const unsigned char ROTATE_3D = 8;
        static const unsigned char MOVE_ROTATE_3D = 9;

        unsigned char interaction_mode;


        bool always_visible;

        std::vector<Marker> markers;

        bool independent_marker_orientation;

        std::string description;

    };
}