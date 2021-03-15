#pragma once

namespace ros {
    class TransportHints {
    public:
        enum class Hint {
            Reliable,
            Unreliable
        } hint;

        TransportHints& reliable() {
            hint = Hint::Reliable;
            return *this;
        }
        TransportHints& unreliable() {
            hint = Hint::Unreliable;
            return *this;
        }
    };
}