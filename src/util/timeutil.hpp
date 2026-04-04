#pragma once

#include <chrono>

#include "typedefs.hpp"

namespace timeutil {
    class Timer {
        std::chrono::high_resolution_clock::time_point start;
    public:
        Timer();
        int64_t stop();
        int64_t stopNs();
    };

    /**
     * Timer that stops and prints time when destructor called
     * @example:
     *     {    // some scope (custom, function, if/else, cycle etc.)
     *     timeutil::ScopeLogTimer scopeclock();
     *     ...
     *     }
     */
    class ScopeLogTimer : public Timer {
        long long scopeid_;
    public:
        ScopeLogTimer(long long id);
        ~ScopeLogTimer();
    };

    inline constexpr float time_value(float hour, float minute, float second) {
        return (hour + (minute + second / 60.0f) / 60.0f) / 24.0f;
    }

    void from_value(float value, int& hour, int& minute, int& second);
}
