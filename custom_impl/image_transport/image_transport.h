#pragma once

#include <ros/node_handle.h>
#include <ros/transport_hints.h>

namespace image_transport {
    struct Exception {
        const char * what() const {
            return "WHAT";
        }
    };

    struct TransportHints{
        TransportHints(std::string) {

        }

        TransportHints(std::string, const ros::TransportHints&) {

        }
    };

    class ImageTransport {
    public:
        ImageTransport(ros::NodeHandle&h) {

        }
    };
}