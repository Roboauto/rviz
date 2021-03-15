#pragma once

#include <string>

#include <impl/conversions.h>


namespace ros {
    namespace message_traits {

        template<class T>
        const char* datatype() {
            return impl::conversions::datatype<impl::conversions::msgAlternative_t<T>>();
        }

        template<class T>
        struct IsMessage;
        /*{
            static const bool value = false;
        };*/
    }
}