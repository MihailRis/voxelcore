#pragma once

#include <functional>

namespace gui {
    enum class Orientation { VERTICAL, HORIZONTAL };
    
    using OnTimeOut = std::function<void()>;

    struct IntervalEvent {
        OnTimeOut callback;
        float interval;
        float timer;
        // -1 - infinity, 1 - one time event
        int repeat;
    };
}
