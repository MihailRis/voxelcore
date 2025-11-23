#pragma once

#include <stdexcept>

namespace util {
    template<typename T, int capacity>
    class stack_vector {
        struct buffer {
            alignas(alignof(T)) char* data[sizeof(T) * capacity];

            T* ptr() {
                return reinterpret_cast<T*>(data);
            }
            const T* ptr() const {
                return reinterpret_cast<const T*>(data);
            }
        };
    public:
        stack_vector() : size_(0) {}

        stack_vector(const stack_vector<T, capacity>& other)
            : size_(other.size_) {
            for (int i = 0; i < size_; ++i) {
                new (&data_.ptr()[i]) T(other.data_.ptr()[i]);
            }
        }

        stack_vector(stack_vector<T, capacity>&& other) noexcept
            : size_(other.size_) {
            for (int i = 0; i < size_; ++i) {
                new (&data_.ptr()[i]) T(std::move(other.data_.ptr()[i]));
            }
            other.size_ = 0;
        }

        stack_vector(const std::initializer_list<T>& init) : size_(0) {
            static_assert(
                init.size() <= capacity,
                "initializer list exceeds stack vector capacity"
            );
            for (const auto& value : init) {
                new (&data_.ptr()[size_++]) T(value);
            }
        }

        ~stack_vector() = default;

        void push_back(const T& value) {
            if (size_ < capacity) {
                auto data = reinterpret_cast<char*>(data_.ptr() + (size_++));
                new (data) T(value);
            } else {
                throw std::overflow_error("stack vector capacity exceeded");
            }
        }

        void push_back(T&& value) {
            if (size_ < capacity) {
                auto data = reinterpret_cast<char*>(data_.ptr() + (size_++));
                new (data) T(std::move(value));
            } else {
                throw std::overflow_error("stack vector capacity exceeded");
            }
        }

        void pop_back() {
            if (size_ > 0) {
                data_.ptr()[size_ - 1].~T();
                --size_;
            } else {
                throw std::underflow_error("stack vector is empty");
            }
        }

        void clear() {
            for (int i = 0; i < size_; ++i) {
                data_.ptr()[i].~T();
            }
            size_ = 0;
        }

        T& operator[](int index) {
            return data_.ptr()[index];
        }
        
        const T& operator[](int index) const {
            return data_.ptr()[index];
        }

        T& at(int index) {
            if (index < 0 || index >= size_) {
                throw std::out_of_range("index out of range");
            }
            return data_.ptr()[index];
        }

        const T& at(int index) const {
            if (index < 0 || index >= size_) {
                throw std::out_of_range("index out of range");
            }
            return data_.ptr()[index];
        }

        int size() const { 
            return size_;
        }

        bool empty() const {
            return size_ == 0;
        }

        bool full() const {
            return size_ == capacity;
        }

        auto begin() {
            return data_.ptr();
        }

        auto end() {
            return data_.ptr() + size_;
        }

        auto begin() const {
            return data_.ptr();
        }

        auto end() const {
            return data_.ptr() + size_;
        }
    private:
        buffer data_;
        int size_;
    };
}
