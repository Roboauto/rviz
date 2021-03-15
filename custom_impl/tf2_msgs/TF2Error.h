#pragma once

namespace tf2_msgs {
    struct TF2Error {
        enum {
            NO_ERROR,
            LOOKUP_ERROR,
            CONNECTIVITY_ERROR,
            EXTRAPOLATION_ERROR,
            INVALID_ARGUMENT_ERROR,
            TIMEOUT_ERROR,
            TRANSFORM_ERROR
        } error{};
        std:: string error_string{};
    };

}