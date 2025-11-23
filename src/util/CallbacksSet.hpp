#pragma once

#include <memory>
#include <vector>
#include <functional>

template<typename... Args>
class CallbacksSet {
public:
    using Func = std::function<void(Args...)>;
private:
    std::unique_ptr<std::vector<Func>> callbacks;
public:
    void listen(const Func& callback) {
        if (callbacks == nullptr) {
            callbacks = std::make_unique<std::vector<Func>>();
        }
        callbacks->push_back(callback);
    }

    void notify(Args&&... args) {
        if (callbacks) {
            for (auto& callback : *callbacks) {
                callback(std::forward<Args>(args)...);
            }
        }
    }
};
