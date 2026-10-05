#pragma once

#include <cassert>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>
#include <utility>

namespace util {
    template<typename T, std::size_t capacity>
    class stack_vector {
        struct buffer {
            alignas(alignof(T)) char data[sizeof(T) * capacity];

            T* ptr() {
                return reinterpret_cast<T*>(data);
            }
            const T* ptr() const {
                return reinterpret_cast<const T*>(data);
            }
        };
    public:
        using size_type = std::size_t;
        stack_vector() : size_(0) {}

        stack_vector(const stack_vector<T, capacity>& other)
            : size_(other.size_) {
            for (size_type i = 0; i < size_; ++i) {
                new (&data_.ptr()[i]) T(other.data_.ptr()[i]);
            }
        }

        stack_vector(stack_vector<T, capacity>&& other) noexcept
            : size_(other.size_) {
            for (size_type i = 0; i < size_; ++i) {
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

        ~stack_vector() {
            clear();
        }

        void push_back(const T& value) {
            if (size_ >= capacity) {
                throw std::overflow_error("stack vector capacity exceeded");
            }

            void* p = data_.data + size_ * sizeof(T);
            ::new (p) T(value);

            ++size_;
        }

        void push_back(T&& value) {
            if (size_ >= capacity) {
                throw std::overflow_error("stack vector capacity exceeded");
            }

            void* p = data_.data + size_ * sizeof(T);
            ::new (p) T(std::move(value));

            ++size_;
        }

        void pop_back() noexcept {
            assert(!empty() && "pop_back() called on empty stack_vector");
            if (size_ > 0) {
                data_.ptr()[size_ - 1].~T();
                --size_;
            } 
        }

        void clear() noexcept {
            for (size_type i = 0; i < size_; ++i) {
                data_.ptr()[i].~T();
            }
            size_ = 0;
        }

        T& operator[](size_type index) {
            return data_.ptr()[index];
        }
        
        const T& operator[](size_type index) const {
            return data_.ptr()[index];
        }

        T& at(size_type index) {
            if (index >= size_) {
                throw std::out_of_range("index out of range");
            }
            return data_.ptr()[index];
        }

        const T& at(size_type index) const {
            if (index >= size_) {
                throw std::out_of_range("index out of range");
            }
            return data_.ptr()[index];
        }

        T& front() {
            assert(!empty() && "front() called on empty stack_vector");
            return data_.ptr()[0];
        }

        T& back() {
            assert(!empty() && "back() called on empty stack_vector");
            return data_.ptr()[size_ - 1];
        }

        const T& front() const {
            assert(!empty() && "front() called on empty stack_vector");
            return data_.ptr()[0];
        }

        const T& back() const {
            assert(!empty() && "back() called on empty stack_vector");
            return data_.ptr()[size_ - 1];
        }

        size_type size() const { 
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
        size_type size_;
    };
}
