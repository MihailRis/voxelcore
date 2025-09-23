#pragma once

#include <stdexcept>

namespace util {
    template<typename T, int capacity>
    class stack_vector {
    public:
        stack_vector() : size_(0) {}

        stack_vector(const stack_vector<T, capacity>& other)
            : size_(other.size_) {
            for (int i = 0; i < size_; ++i) {
                new (&data_[i]) T(other.data_[i]);
            }
        }

        stack_vector(stack_vector<T, capacity>&& other) noexcept
            : size_(other.size_) {
            for (int i = 0; i < size_; ++i) {
                new (&data_[i]) T(std::move(other.data_[i]));
            }
            other.size_ = 0;
        }

        stack_vector(const std::initializer_list<T>& init) : size_(0) {
            static_assert(
                init.size() <= capacity,
                "initializer list exceeds stack vector capacity"
            );
            for (const auto& value : init) {
                new (&data_[size_++]) T(value);
            }
        }

        ~stack_vector() {
            clear();
        }

        void push_back(const T& value) {
            if (size_ < capacity) {
                data_[size_++] = value;
            } else {
                throw std::overflow_error("stack vector capacity exceeded");
            }
        }

        void pop_back() {
            if (size_ > 0) {
                data_[size_ - 1].~T();
                --size_;
            } else {
                throw std::underflow_error("stack vector is empty");
            }
        }

        void clear() {
            for (int i = 0; i < size_; ++i) {
                data_[i].~T();
            }
            size_ = 0;
        }

        T& operator[](int index) {
            return data_[index];
        }
        
        const T& operator[](int index) const {
            return data_[index];
        }

        T& at(int index) {
            if (index < 0 || index >= size_) {
                throw std::out_of_range("index out of range");
            }
            return data_[index];
        }

        const T& at(int index) const {
            if (index < 0 || index >= size_) {
                throw std::out_of_range("index out of range");
            }
            return data_[index];
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
            return data_;
        }

        auto end() {
            return data_ + size_;
        }

        auto begin() const {
            return data_;
        }

        auto end() const {
            return data_ + size_;
        }
    private:
        T data_[capacity];
        int size_;
    };
}
