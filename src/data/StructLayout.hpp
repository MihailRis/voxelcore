#pragma once

#include <vector>
#include <string>
#include <stdexcept>
#include <unordered_map>

#include "typedefs.hpp"
#include "interfaces/Serializable.hpp"

namespace data {
    enum class FieldType {
        I8=0, I16, I32, I64, F32, F64, CHAR
    };

    inline std::string to_string(FieldType type) {
        const char* names[] = {
            "int8", "int16", "int32", "int64", "float32", "float64", "char" 
        };
        return names[static_cast<int>(type)];
    }
    FieldType FieldType_from_string(std::string_view name);

    /// @brief Sorted by severity
    enum class FieldIncapatibilityType {
        NONE = 0,
        DATA_LOSS,
        TYPE_ERROR,
        MISSING,
    };
    inline const char* to_string(FieldIncapatibilityType type) {
        const char* names[] = {
            "none", "data_loss", "type_error", "missing"
        };
        return names[static_cast<int>(type)];
    }

    struct FieldIncapatibility {
        std::string name;
        FieldIncapatibilityType type;
    };

    inline constexpr int sizeof_type(FieldType type) {
        const int sizes[] = {
            1, 2, 4, 8, 4, 8, 1
        };
        return sizes[static_cast<int>(type)];
    }

    class dataloss_error : public std::runtime_error {
    public:
        dataloss_error(const std::string& message) : std::runtime_error(message) {}
    };

    /// @brief Strategy will be used if value can't be left the same on conversion
    enum class FieldConvertStrategy {
        /// @brief Reset field value
        RESET = 0,
        /// @brief Clamp field value if out of type range
        CLAMP
    };
    
    inline const char* to_string(FieldConvertStrategy strategy) {
        const char* names[] = {
            "reset", "clamp"
        };
        return names[static_cast<int>(strategy)];
    }
    FieldConvertStrategy FieldConvertStrategy_from_string(std::string_view name);

    struct Field {
        FieldType type;
        std::string name;
        /// @brief Number of field elements (array size)
        int elements;
        /// @brief Strategy will be used in data loss case
        FieldConvertStrategy convertStrategy;
        /// @brief Byte offset of the field
        int offset;
        /// @brief Byte size of the field
        int size;

        Field(
            FieldType type, 
            std::string name, 
            int elements, 
            FieldConvertStrategy strategy=FieldConvertStrategy::RESET
        ) : type(type), 
            name(std::move(name)), 
            elements(elements), 
            convertStrategy(strategy),
            offset(0),
            size(0) {}

        bool operator==(const Field& o) const {
            return type == o.type && 
                   name == o.name && 
                   elements == o.elements &&
                   convertStrategy == o.convertStrategy &&
                   offset == o.offset &&
                   size == o.size;
        }
        bool operator!=(const Field& o) const {
            return !operator==(o);
        }
    };
    
    class StructLayout : public Serializable {
        int totalSize;
        std::vector<Field> fields;
        std::unordered_map<std::string, int> indices;
    
        StructLayout(
            int totalSize,
            std::vector<Field> fields,
            std::unordered_map<std::string, int> indices
        ) : totalSize(totalSize), 
            fields(std::move(fields)),
            indices(std::move(indices))
        {}
    public:
        StructLayout() : StructLayout(0, {}, {}) {}

        bool operator==(const StructLayout& o) const {
            // if fields are completely equal then structures are equal
            return fields == o.fields;
        }
        bool operator!=(const StructLayout& o) const {
            return !operator==(o);
        }

        /// @brief Get field by name. Returns nullptr if field not found.
        /// @param name field name
        /// @return nullable field pointer
        [[nodiscard]]
        const Field* getField(const std::string& name) const {
            auto found = indices.find(name);
            if (found == indices.end()) {
                return nullptr;
            }
            return &fields.at(found->second);
        }

        /// @brief Get field by name
        /// @throws std::runtime_exception - field not found
        /// @param name field name
        /// @return read-only field reference
        const Field& requireField(const std::string& name) const;

        [[nodiscard]]
        integer_t getInteger(const ubyte* src, const std::string& name, int index=0) const {
            return getInteger(src, requireField(name), index);
        }

        /// @brief Get integer from specified field. 
        /// Types: (i8, i16, i32, i64, char)
        /// @throws std::runtime_exception - field not found
        /// @throws std::out_of_range - index is out of range [0, elements-1]
        /// @param src source buffer
        /// @param field target field
        /// @param index array index
        /// @return field value
        [[nodiscard]]
        integer_t getInteger(const ubyte* src, const Field& field, int index=0) const;

        [[nodiscard]]
        number_t getNumber(const ubyte* src, const std::string& name, int index=0) const {
            return getNumber(src, requireField(name), index);
        }

        /// @brief Get floating-point number from specified field. 
        /// Types: (f32, f64, i8, i16, i32, i64, char)
        /// @throws std::runtime_exception - field not found
        /// @throws std::out_of_range - index is out of range [0, elements-1]
        /// @param src source buffer
        /// @param field target field
        /// @param index array index
        /// @return field value
        [[nodiscard]]
        number_t getNumber(const ubyte* src, const Field& field, int index=0) const;

        [[nodiscard]]
        std::string_view getChars(const ubyte* src, const std::string& name) const {
            return getChars(src, requireField(name));
        }

        /// @brief Get read-only chars array as string_view.
        /// @param src source buffer
        /// @param field target field
        [[nodiscard]]
        std::string_view getChars(const ubyte* src, const Field& field) const;

        void setInteger(ubyte* dst, integer_t value, const std::string& name, int index=0) const {
            setInteger(dst, value, requireField(name), index);
        }

        /// @brief Set field integer value.
        /// Types: (i8, i16, i32, i64, f32, f64, char)
        /// @throws std::runtime_exception - field not found
        /// @throws std::out_of_range - index is out of range [0, elements-1]
        /// @param dst destination buffer
        /// @param value value
        /// @param field target field
        /// @param index array index
        void setInteger(ubyte* dst, integer_t value, const Field& field, int index=0) const;

        void setNumber(ubyte* dst, number_t value, const std::string& name, int index=0) const {
            setNumber(dst, value, requireField(name), index);
        }

        /// @brief Set field numeric value.
        /// Types: (f32, f64)
        /// @throws std::runtime_exception - field not found
        /// @throws std::out_of_range - index is out of range [0, elements-1]
        /// @param dst destination buffer
        /// @param value value
        /// @param field target field
        /// @param index array index
        void setNumber(ubyte* dst, number_t value, const Field& field, int index=0) const;
        
        /// @brief Replace chars array to given ASCII string
        /// @throws std::runtime_exception - field not found
        /// @see StructMapper::setUnicode - utf-8 version of setAscii
        /// @param dst destination buffer
        /// @param value ASCII string
        /// @param name field name
        /// @return number of written string chars
        size_t setAscii(ubyte* dst, std::string_view value, const std::string& name) const;

        size_t setUnicode(ubyte* dst, std::string_view value, const std::string& name) const {
            return setUnicode(dst, value, requireField(name));
        }
        
        /// @brief Unicode-safe version of setAscii
        /// @throws std::runtime_exception - field not found
        /// @param dst destination buffer
        /// @param value utf-8 string
        /// @param name field name
        /// @return number of written string chars
        size_t setUnicode(ubyte* dst, std::string_view value, const Field& field) const;

        /// @return total structure size (bytes)
        [[nodiscard]] size_t size() const {
            return totalSize;
        }

        [[nodiscard]] auto begin() const {
            return fields.begin();
        }

        [[nodiscard]] auto end() const {
            return fields.end();
        }

        /// @brief Convert structure data from srcLayout to this layout.
        /// @param srcLayout source structure layout
        /// @param src source data
        /// @param dst destination buffer
        /// (size must be enough to store converted structure) 
        /// @param allowDataLoss allow to drop fields that are not present in 
        /// this layout or have incompatible types
        /// @throws data::dataloss_error - data loss detected and allowDataLoss 
        /// is set to false
        void convert(
            const StructLayout& srcLayout, 
            const ubyte* src, 
            ubyte* dst,
            bool allowDataLoss) const;

        std::vector<FieldIncapatibility> checkCompatibility(
            const StructLayout& dstLayout);

        [[nodiscard]]
        static StructLayout create(const std::vector<Field>& fields);

        dv::value serialize() const override;
        void deserialize(const dv::value& src) override;
    };
}
