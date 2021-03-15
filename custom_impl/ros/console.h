#pragma once

#include "package.h"

#define ROSCONSOLE_DEFAULT_NAME "ROSCONSOLE"

namespace ros {
    namespace console {
        namespace levels {
            enum Level {
                Error = 0,
                Warn = 1,
                Info = 2,
                Debug = 3
            };
        }
    }
}