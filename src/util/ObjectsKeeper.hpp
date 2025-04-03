#pragma once

#include <memory>
#include <vector>

#include "util/observer_handler.hpp"

namespace util {
    /// @brief Keeps shared pointers alive until destruction
    class ObjectsKeeper {
        std::vector<std::shared_ptr<void>> ptrs;
        std::vector<ObserverHandler> handlers;
    public:
        ObjectsKeeper() = default;

        ObjectsKeeper(const ObjectsKeeper&) = delete;

        ObjectsKeeper(ObjectsKeeper&& keeper) noexcept
            : ptrs(std::move(keeper.ptrs)),
              handlers(std::move(keeper.handlers)) {
        }

        ObjectsKeeper& operator=(ObjectsKeeper&& keeper) noexcept = default;

        virtual ~ObjectsKeeper() {
        }

        virtual void keepAlive(std::shared_ptr<void> ptr) {
            ptrs.push_back(std::move(ptr));
        }

        virtual void keepAlive(ObserverHandler&& ptr) {
            handlers.emplace_back(std::move(ptr));
        }

        virtual void clearKeepedObjects() {
            ptrs.clear();
            handlers.clear();
        }
    };
}
