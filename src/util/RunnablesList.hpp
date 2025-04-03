#pragma once

#include <mutex>
#include <memory>
#include <unordered_map>
#include <utility>

#include "delegates.hpp"
#include "typedefs.hpp"

namespace util {
    class RunnablesList {
        int nextid = 1;
        std::unordered_map<int, runnable> runnables;
        std::mutex mutex;
    public:
        RunnablesList() = default;

        RunnablesList(RunnablesList&& o) {
            runnables = std::move(o.runnables);
            nextid = o.nextid;
        }

        void operator=(RunnablesList&& o) {
            runnables = std::move(o.runnables);
            nextid = o.nextid;
        }

        ObserverHandler add(runnable callback) {
            std::lock_guard lock(mutex);
            int id = nextid++;
            runnables[id] = std::move(callback);
            return ObserverHandler(new int(id), [this](int* id) { //-V508
                std::lock_guard lock(mutex);
                runnables.erase(*id);
                delete id;
            });
        }

        void notify() {
            std::lock_guard lock(mutex);
            for (auto& entry : runnables) {
                entry.second();
            }
        }
    };
}
