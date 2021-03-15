#pragma once

namespace KDL {
    namespace Rotation {

        struct Quaternion {
            template<class ...T>
            Quaternion(T... args) {

            }
        };
    }

    struct Vector {
        template<class ...T>
        Vector(T... args) {

        }

    };
    struct Frame {
        template<class ...T>
        Frame(T... args) {

        }

    };
}