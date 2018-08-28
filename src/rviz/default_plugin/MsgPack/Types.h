#pragma once
//
// Created by martin on 8/27/18.
//
#pragma once

#include <msgpack.hpp>

namespace RoboCore{
    struct PoseXYZ{
        float x, y, z;

        MSGPACK_DEFINE(x,y,z)
    };

    struct PoseXYZI{
        float x, y, z, i;

        MSGPACK_DEFINE(x,y,z,i)
    };

    struct Scale{
        float x=0.1, y=0.1, z=0.1;

        MSGPACK_DEFINE(x,y,z)
    };

    struct Color{
        float r=1,g=1,b=1,a=1;

        MSGPACK_DEFINE(r,g,b,a)
    };

    struct Orientation{
        float x,y,z,w=1.0;

        MSGPACK_DEFINE(x,y,z,w)
    };

    struct Properties{
    public:
        Properties(Color color, Scale scale, std::string frame, Orientation orientation=Orientation()) :
                color_(color),
                scale_(scale),
                orientation_(orientation),
                frame_(std::move(frame))
        {

        }

        Color color_{};
        Scale scale_{};
        Orientation orientation_{};
        std::string frame_;
    };
}
