#pragma once

#include <nav_msgs/GridCells.h>
#include <MQTTVisualizationMessages2/GridCellsMsg.h>

#include "time.h"

namespace impl { namespace conversions {
    template<>
    struct MsgAlternative<nav_msgs::GridCells> {
        using type = MQTTVisualizationMessages2::GridCells;
    };

    template<>
    struct ROSConvertor<MQTTVisualizationMessages2::GridCells> {
        static inline nav_msgs::GridCells convert(const MQTTVisualizationMessages2::GridCells& t) {
            return {};
        }
    };
}}