#pragma once

#include <cstdint>
#include <cstring>
#include <vector>
#include <limits>
#include <stdexcept>

#include "Buffer.hpp"
#include "data_io.hpp"

namespace util {    
    template<typename T>
    inline T read_int_le(const uint8_t* src) {
        T tmp;
        std::memcpy(&tmp, src, sizeof(T));
        return dataio::le2h(tmp);
    }

    /// @brief Simple heap implementation for memory-optimal sparse array of 
    /// small different structures
    /// @note alignment is not impemented 
    /// (impractical in the context of scripting and memory consumption)
    /// @tparam Tindex entry index type
    /// @tparam Tsize entry size type
    template <typename Tindex, typename Tsize>
    class SmallHeap {
        std::vector<uint8_t> buffer;
        Tindex entriesCount;
    public:
        SmallHeap() : entriesCount(0) {}

        /// @brief Find current entry address by index
        /// @param index entry index
        /// @return temporary raw pointer or nullptr if entry does not exists
        /// @attention pointer becomes invalid after allocate(...) or free(...)
        uint8_t* find(Tindex index) {
            auto data = buffer.data();
            for (size_t i = 0; i < entriesCount; i++) {
                auto nextIndex = read_int_le<Tindex>(data);
                data += sizeof(Tindex);
                auto nextSize = read_int_le<Tsize>(data);
                data += sizeof(Tsize);
                if (nextIndex == index) {
                    return data;
                } else if (nextIndex > index) {
                    return nullptr;
                }
                data += nextSize;
            }
            return nullptr;
        }

        /// @brief Erase entry from the heap
        /// @param ptr valid entry pointer
        void free(uint8_t* ptr) {
            if (ptr == nullptr) {
                return;
            }
            auto entrySize = sizeOf(ptr);
            auto begin =
                buffer.begin() +
                ((ptr - sizeof(Tsize) - sizeof(Tindex)) - buffer.data());
            buffer.erase(
                begin, begin + entrySize + sizeof(Tsize) + sizeof(Tindex)
            );
            entriesCount--;
        }

        /// @brief Create or update entry (size)
        /// @param index entry index
        /// @param size entry size
        /// @return temporary entry pointer 
        /// @attention pointer becomes invalid after allocate(...) or free(...)
        uint8_t* allocate(Tindex index, size_t size) {
            const auto maxSize = std::numeric_limits<Tsize>::max();
            if (size > maxSize) {
                throw std::invalid_argument(
                    "requested "+std::to_string(size)+" bytes but limit is "+
                    std::to_string(maxSize));
            }
            if (size == 0) {
                throw std::invalid_argument("zero size");
            }
            ptrdiff_t offset = 0;
            if (auto found = find(index)) {
                auto entrySize = sizeOf(found);
                if (size == entrySize) {
                    std::memset(found, 0, entrySize);
                    return found;
                }
                free(found);
            }
            for (size_t i = 0; i < entriesCount; i++) {
                auto data = buffer.data() + offset;
                auto nextIndex = read_int_le<Tindex>(data);
                data += sizeof(Tindex);
                auto nextSize = read_int_le<Tsize>(data);
                data += sizeof(Tsize);
                if (nextIndex > index) {
                    break;
                }
                data += nextSize;
                offset = data - buffer.data();
            }
            buffer.insert(
                buffer.begin() + offset,
                size + sizeof(Tindex) + sizeof(Tsize),
                0
            );
            entriesCount++;

            auto data = buffer.data() + offset;
            
            Tindex indexTmp = dataio::h2le(index);
            Tsize sizeTmp = dataio::h2le(size);
            std::memcpy(data, &indexTmp, sizeof(Tindex));
            data += sizeof(Tindex);
            std::memcpy(data, &sizeTmp, sizeof(Tsize));
            return data + sizeof(Tsize);
        }

        /// @param ptr valid entry pointer
        /// @return entry size
        Tsize sizeOf(uint8_t* ptr) const {
            if (ptr == nullptr) {
                return 0;
            }
            return read_int_le<Tsize>(ptr - sizeof(Tsize));
        }

        /// @return number of entries
        Tindex count() const {
            return entriesCount;
        }

        /// @return total used bytes including entries metadata
        size_t size() const {
            return buffer.size();
        }

        inline bool operator==(const SmallHeap<Tindex, Tsize>& o) const {
            if (o.entriesCount != entriesCount) {
                return false;
            }
            return buffer == o.buffer;
        }

        util::Buffer<uint8_t> serialize() const {
            util::Buffer<uint8_t> out(sizeof(Tindex) + buffer.size());
            ubyte* dst = out.data();
            const ubyte* src = buffer.data();

            Tindex countTmp = dataio::h2le(entriesCount);
            std::memcpy(dst, &countTmp, sizeof(Tindex));
            dst += sizeof(Tindex);

            std::memcpy(dst, src, buffer.size());
            return out;
        }

        void deserialize(const ubyte* src, size_t size) {
            entriesCount = read_int_le<Tindex>(src);
            buffer.resize(size - sizeof(Tindex));
            std::memcpy(buffer.data(), src + sizeof(Tindex), buffer.size());
        }

        struct const_iterator {
        private:
            const std::vector<uint8_t>& buffer;
        public:
            Tindex index;
            size_t offset;

            const_iterator(
                const std::vector<uint8_t>& buffer, Tindex index, size_t offset
            ) : buffer(buffer), index(index), offset(offset) {}

            Tsize size() const {
                return read_int_le<Tsize>(buffer.data() + offset - sizeof(Tsize));
            }

            bool operator!=(const const_iterator& o) const {
                return o.offset != offset;
            }

            const_iterator& operator++() {
                offset += size();
                if (offset == buffer.size()) {
                    return *this;
                }
                index = read_int_le<Tindex>(buffer.data() + offset);
                offset += sizeof(Tindex) + sizeof(Tsize);
                return *this;
            }

            const_iterator& operator*() {
                return *this;
            }

            const uint8_t* data() const {
                return buffer.data() + offset;
            }
        };

        const_iterator begin() const {
            if (buffer.empty()) {
                return end();
            }
            return const_iterator (
                buffer,
                read_int_le<Tindex>(buffer.data()),
                sizeof(Tindex) + sizeof(Tsize));
        }

        const_iterator end() const {
            return const_iterator (buffer, 0, buffer.size());
        }
    };
}
