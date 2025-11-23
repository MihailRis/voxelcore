#pragma once

#include <stdexcept>

namespace util {
    template <typename T>
    class span {
    public:
        constexpr span(const T* ptr, size_t length)
         : ptr(ptr), length(length) {}

        const T& operator[](size_t index) const {
            return ptr[index];
        }

        const T& at(size_t index) const {
            if (index >= length) {
                throw std::out_of_range("index is out of range");
            }
            return ptr[index];
        }

        auto begin() const {
            return ptr;
        }

        auto end() const {
            return ptr + length;
        }

        const T* data() const {
            return ptr;
        }

        size_t size() const {
            return length;
        }
    private:
        const T* ptr;
        size_t length;
    };
}
