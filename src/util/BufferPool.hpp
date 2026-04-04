#pragma once

#include <memory>
#include <mutex>
#include <queue>
#include <vector>

namespace util {
    /// @brief Thread-safe pool of same-sized buffers
    /// @tparam T array type
    template <class T>
    class BufferPool {
        std::vector<std::unique_ptr<T[]>> buffers;
        std::queue<T*> freeBuffers;
        std::mutex mutex;
        size_t bufferSize;
    public:
        BufferPool(size_t bufferSize) : bufferSize(bufferSize) {
        }

        /// @brief Retrieve a buffer for use
        /// @return pointer that brings buffer back to the pool when destroyed
        std::shared_ptr<T[]> get() {
            std::lock_guard lock(mutex);
            if (freeBuffers.empty()) {
                buffers.emplace_back(std::make_unique<T[]>(bufferSize));
                freeBuffers.push(buffers[buffers.size() - 1].get());
            }
            auto* buffer = freeBuffers.front();
            freeBuffers.pop();
            return std::shared_ptr<T[]>(buffer, [this](T* ptr) {
                std::lock_guard lock(mutex);
                freeBuffers.push(ptr);
            });
        }

        size_t getBufferSize() const {
            return bufferSize;
        }
    };
}
