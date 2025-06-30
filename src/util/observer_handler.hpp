#pragma once

#include <functional>

class ObserverHandler {
public:
    ObserverHandler() = default;

    ObserverHandler(std::function<void()> destructor)
        : destructor(std::move(destructor)) {
    }

    ObserverHandler(const ObserverHandler&) = delete;

    ObserverHandler(ObserverHandler&& handler) noexcept
        : destructor(std::move(handler.destructor)) {
        handler.destructor = nullptr;
    }

    ~ObserverHandler() {
        if (destructor) {
            destructor();
        }
    }

    bool operator==(std::nullptr_t) const {
        return destructor == nullptr;
    }

    ObserverHandler& operator=(const ObserverHandler& handler) = delete;

    ObserverHandler& operator=(ObserverHandler&& handler) noexcept {
        if (destructor) {
            destructor();
        }
        destructor = std::move(handler.destructor);
        handler.destructor = nullptr;
        return *this;
    }
private:
    std::function<void()> destructor;
};
