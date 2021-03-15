#pragma once

#include "Marker.h"
#include <memory>

namespace visualization_msgs {
    struct MarkerArray {
        using ConstPtr = boost::shared_ptr<const MarkerArray>;
        std::vector<Marker> markers;
    };
}

#include <impl/conversions/markerArray.h> // I dont want to modify ros files, so this is only solution i found