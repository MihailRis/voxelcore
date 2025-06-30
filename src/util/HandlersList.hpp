#pragma once

#include <algorithm>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "delegates.hpp"
#include "typedefs.hpp"
#include "util/observer_handler.hpp"

namespace util {
    template <class... Types>
    class HandlersList {
        int nextid = 1;
        std::unordered_map<int, std::function<bool(Types...)>> handlers;
        std::vector<int> order;
        std::mutex mutex;
    public:
        HandlersList() = default;

        HandlersList(HandlersList&& o) {
            handlers = std::move(o.handlers);
            order = std::move(o.order);
            nextid = o.nextid;
        }

        void operator=(HandlersList&& o) {
            handlers = std::move(o.handlers);
            order = std::move(o.order);
            nextid = o.nextid;
        }

        ObserverHandler add(std::function<bool(Types...)> handler) {
            std::lock_guard lock(mutex);
            int id = nextid++;
            handlers[id] = std::move(handler);
            order.push_back(id);
            return ObserverHandler([this, id]() {
                std::lock_guard lock(mutex);
                handlers.erase(id);
                order.erase(
                    std::remove(order.begin(), order.end(), id), order.end()
                );
            });
        }

        void notify(Types... args) {
            std::vector<int> orderCopy;
            decltype(handlers) handlersCopy;
            {
                std::lock_guard lock(mutex);
                orderCopy = order;
                handlersCopy = handlers;
            }
            for (auto it = orderCopy.rbegin(); it != orderCopy.rend(); ++it) {
                if (handlersCopy.at(*it)(args...)) {
                    break;
                }
            }
        }
    };
}
