#pragma once

namespace impl { namespace conversions {

    template<class A>
    struct MsgAlternative;

    template<class A>
    struct ROSConvertor;

    template<class A>
    inline const char* datatype() {
        return A::typeName;
    }

    template<class A>
    using msgAlternative_t = typename MsgAlternative<A>::type;

    template<class A>
    auto toRos(const A& x) {
        return ROSConvertor<A>::convert(x);
    }


} }