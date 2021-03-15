#pragma once

#include <exception>

namespace ros {
    struct Exception{
        Exception(const char *x): x_{x} {

        }
        const char * what() const {
            return x_;
        }
    protected:
        const char * x_;
    };
}