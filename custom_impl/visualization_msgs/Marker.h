#pragma once

#include <std_msgs/Header.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/ColorRGBA.h>

#include <vector>
#include <memory>

namespace visualization_msgs {
    struct Marker {
        using ConstPtr = boost::shared_ptr<const Marker>;
        using Ptr = boost::shared_ptr<Marker>;

        static const unsigned char ARROW = 0;
        static const unsigned char CUBE = 1;
        static const unsigned char SPHERE = 2;
        static const unsigned char CYLINDER = 3;
        static const unsigned char LINE_STRIP = 4;
        static const unsigned char LINE_LIST = 5;
        static const unsigned char CUBE_LIST = 6;
        static const unsigned char SPHERE_LIST = 7;
        static const unsigned char POINTS = 8;
        static const unsigned char TEXT_VIEW_FACING = 9;
        static const unsigned char MESH_RESOURCE = 10;
        static const unsigned char TRIANGLE_LIST = 11;

        static const unsigned char ADD = 0;
        static const unsigned char MODIFY = 0;
        static const unsigned char DELETE = 2;
        static const unsigned char DELETEALL = 3;

        std_msgs::Header header;
        std::string ns;
        int id;
        int type;
        int action;
        geometry_msgs::Pose pose;
        geometry_msgs::Vector3 scale;
        std_msgs::ColorRGBA color;
        ros::Duration lifetime;
        bool frame_locked;

        std::vector <geometry_msgs::Point> points;
        std::vector <std_msgs::ColorRGBA> colors;

        std::string text;

        std::string mesh_resource;
        bool mesh_use_embedded_materials;
    };
}

#include <impl/conversions/marker.h> // I dont want to modify ros files, so this is only solution i found