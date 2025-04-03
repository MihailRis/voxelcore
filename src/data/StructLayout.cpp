#include "StructLayout.hpp"

#include <map>
#include <cstring>
#include <climits>
#include <string.h>
#include <algorithm>

#include "data/dv.hpp"
#include "util/data_io.hpp"
#include "util/stringutil.hpp"

using namespace data;

static_assert(sizeof(float) == sizeof(int32_t));
static_assert(sizeof(double) == sizeof(int64_t));

FieldType data::FieldType_from_string(std::string_view name) {
    std::map<std::string_view, FieldType> map {
        {"int8", FieldType::I8},
        {"int16", FieldType::I16},
        {"int32", FieldType::I32},
        {"int64", FieldType::I64},
        {"float32", FieldType::F32},
        {"float64", FieldType::F64},
        {"char", FieldType::CHAR},
    };
    return map.at(name);
}

FieldConvertStrategy data::FieldConvertStrategy_from_string(std::string_view name) {
    std::map<std::string_view, FieldConvertStrategy> map {
        {"reset", FieldConvertStrategy::RESET},
        {"clamp", FieldConvertStrategy::CLAMP}
    };
    return map.at(name);
}

StructLayout StructLayout::create(const std::vector<Field>& fields) {
    std::vector<Field> builtFields = fields;
    std::unordered_map<std::string, int> indices;

    for (Field& field : builtFields) {
        field.size = sizeof_type(field.type) * field.elements;
    }
    std::sort(builtFields.begin(), builtFields.end(), 
        [](const Field& a, const Field& b) {
            if (a.size == b.size) {
                return a.name < b.name;
            }
            return a.size > b.size;
        }
    );
    int offset = 0;
    for (int i = 0; i < builtFields.size(); i++) {
        auto& field = builtFields[i];
        field.offset = offset;
        indices[field.name] = i;
        offset += field.size;
    }
    return StructLayout(
        offset, std::move(builtFields), std::move(indices));
}

static inline constexpr bool is_integer_type(FieldType type) {
    return (type >= FieldType::I8 && type <= FieldType::I64) || 
            type == FieldType::CHAR;
}

static inline constexpr bool is_floating_point_type(FieldType type) {
    return type == FieldType::F32 || type == FieldType::F64;
}

static inline constexpr bool is_numeric_type(FieldType type) {
    return is_floating_point_type(type) || is_integer_type(type);
}

static inline FieldIncapatibilityType checkIncapatibility(
    const Field& srcField, const Field& dstField
) {
    auto type = FieldIncapatibilityType::NONE;
    if (dstField.elements < srcField.elements) {
        type = FieldIncapatibilityType::DATA_LOSS;
    }
    if (srcField.type == dstField.type) {
        return type;
    }
    if (is_numeric_type(srcField.type) && is_numeric_type(dstField.type)) {
        int sizediff =
            sizeof_type(dstField.type) - sizeof_type(srcField.type);
        if (sizediff < 0) {
            type = std::max(type, FieldIncapatibilityType::DATA_LOSS);
        }
    } else {
        type = std::max(type, FieldIncapatibilityType::TYPE_ERROR);
    }
    return type;
}

static inline integer_t clamp_value(integer_t value, FieldType type) {
    auto typesize = sizeof_type(type) * CHAR_BIT;
    integer_t minval = -(1LL << (typesize-1));
    integer_t maxval = (1LL << (typesize-1))-1;
    return std::min(maxval, std::max(minval, value));
}

static void reset_integer(
    const StructLayout& srcLayout,
    const StructLayout& dstLayout,
    const Field& field,
    const Field& dstField,
    const ubyte* src, 
    ubyte* dst
) {
    int elements = std::min(field.elements, dstField.elements);
    for (int i = 0; i < elements; i++) {
        auto value = srcLayout.getInteger(src, field.name, i);
        auto clamped = clamp_value(value, dstField.type);
        if (dstField.convertStrategy == FieldConvertStrategy::CLAMP) {
            value = clamped;
        } else {
            if (clamped != value) {
                value = 0;
            }
        }
        dstLayout.setInteger(dst, value, field.name, i);
    }
}

static void reset_number(
    const StructLayout& srcLayout,
    const StructLayout& dstLayout,
    const Field& field,
    const Field& dstField,
    const ubyte* src, 
    ubyte* dst
) {
    int elements = std::min(field.elements, dstField.elements);
    for (int i = 0; i < elements; i++) {
        auto value = srcLayout.getNumber(src, field.name, i);
        dstLayout.setNumber(dst, value, field.name, i);
    }
}

void StructLayout::convert(
    const StructLayout& srcLayout, 
    const ubyte* src, 
    ubyte* dst,
    bool allowDataLoss
) const {
    std::memset(dst, 0, totalSize);
    for (const Field& field : srcLayout.fields) {
        auto dstField = getField(field.name);
        if (dstField == nullptr) {
            continue;
        }
        auto type = checkIncapatibility(field, *dstField);
        if (type == FieldIncapatibilityType::TYPE_ERROR) {
            continue;
        }
        // can't just memcpy, because field type may be changed without data loss
        if (is_integer_type(field.type) ||
                (is_floating_point_type(field.type) && 
                    is_integer_type(dstField->type))) {
            reset_integer(srcLayout, *this, field, *dstField, src, dst);
        } else if (is_floating_point_type(dstField->type)) {
            reset_number(srcLayout, *this, field, *dstField, src, dst);
        }
    }
}

std::vector<FieldIncapatibility> StructLayout::checkCompatibility(
    const StructLayout& dstLayout
) {
    std::vector<FieldIncapatibility> report;
    for (const Field& field : fields) {
        auto dstField = dstLayout.getField(field.name);
        if (dstField == nullptr) {
            report.push_back({field.name, FieldIncapatibilityType::MISSING});
            continue;
        }
        auto type = checkIncapatibility(field, *dstField);
        if (type != FieldIncapatibilityType::NONE) {
            report.push_back({field.name, type});
        }
    }
    return report;
}

const Field& StructLayout::requireField(const std::string& name) const {
    auto found = indices.find(name);
    if (found == indices.end()) {
        throw std::runtime_error("field '"+name+"' does not exist");
    }
    return *&fields.at(found->second);
}


template<typename T>
static void set_int(ubyte* dst, integer_t value) {
    T out_value = static_cast<T>(value);
    out_value = dataio::le2h(out_value);
    *reinterpret_cast<T*>(dst) = out_value;
}

void StructLayout::setInteger(
    ubyte* dst, integer_t value, const Field& field, int index
) const {
    if (index < 0 || index >= field.elements) {
        throw std::out_of_range(
            "index out of bounds [0, "+std::to_string(field.elements)+"]");
    }
    auto ptr = dst + field.offset + index * sizeof_type(field.type);
    switch (field.type) {
        case FieldType::I8: set_int<int8_t>(ptr, value); break;
        case FieldType::I16: set_int<int16_t>(ptr, value); break;
        case FieldType::I32: set_int<int32_t>(ptr, value); break;
        case FieldType::I64: set_int<int64_t>(ptr, value); break;
        case FieldType::CHAR: set_int<int8_t>(ptr, value); break;
        case FieldType::F32:
        case FieldType::F64:
            setNumber(dst, static_cast<number_t>(value), field, index);
            break;
        default: 
            throw std::runtime_error("type error");
    }
}

void StructLayout::setNumber(
    ubyte* dst, number_t value, const Field& field, int index
) const {
    if (index < 0 || index >= field.elements) {
        throw std::out_of_range(
            "index out of bounds [0, "+std::to_string(field.elements)+"]");
    }
    auto ptr = dst + field.offset + index * sizeof_type(field.type);
    switch (field.type) {
        case FieldType::F32: {
            float fval = static_cast<float>(value);
            int32_t ival;
            std::memcpy(&ival, &fval, sizeof(int32_t));
            set_int<int32_t>(ptr, ival);
            break;
        }
        case FieldType::F64: {
            double fval = static_cast<double>(value);
            int64_t ival;
            std::memcpy(&ival, &fval, sizeof(int64_t));
            set_int<int64_t>(ptr, ival);
            break;
        }
        default:
            throw std::runtime_error("type error");
    }
}

size_t StructLayout::setAscii(
    ubyte* dst, std::string_view value, const std::string& name
) const {
    const auto& field = requireField(name);
    if (field.type != FieldType::CHAR) {
        throw std::runtime_error("'char' field type required");
    }
    auto ptr = reinterpret_cast<char*>(dst + field.offset);
    auto size = std::min(value.size(), static_cast<std::size_t>(field.elements));
    std::memcpy(ptr, value.data(), size);
    if (size < field.elements) {
        std::memset(ptr + size, 0, field.elements - size);
    }
    return size;
}

size_t StructLayout::setUnicode(
    ubyte* dst, std::string_view value, const Field& field
) const {
    if (field.type != FieldType::CHAR) {
        throw std::runtime_error("'char' field type required");
    }
    auto text = std::string_view(value.data(), value.size());
    size_t size = util::crop_utf8(text, field.elements);
    auto ptr = reinterpret_cast<char*>(dst + field.offset);
    std::memcpy(ptr, value.data(), size);
    if (size < field.elements) {
        std::memset(ptr + size, 0, field.elements - size);
    }
    return size;
}

template<typename T>
static T get_int(const ubyte* src) {
    return dataio::le2h(*reinterpret_cast<const T*>(src));
}

integer_t StructLayout::getInteger(
    const ubyte* src, const Field& field, int index
) const {
    if (index < 0 || index >= field.elements) {
        throw std::out_of_range(
            "index out of bounds [0, "+std::to_string(field.elements)+"]");
    }
    auto ptr = src + field.offset + index * sizeof_type(field.type);
    switch (field.type) {
        case FieldType::I8: return get_int<int8_t>(ptr);
        case FieldType::I16: return get_int<int16_t>(ptr);
        case FieldType::I32: return get_int<int32_t>(ptr);
        case FieldType::I64: return get_int<int64_t>(ptr);
        case FieldType::CHAR: return get_int<int8_t>(ptr);
        default:
            throw std::runtime_error("type error");
    }
}

number_t StructLayout::getNumber(
    const ubyte* src, const Field& field, int index
) const {
    if (index < 0 || index >= field.elements) {
        throw std::out_of_range(
            "index out of bounds [0, "+std::to_string(field.elements)+"]");
    }
    auto ptr = src + field.offset + index * sizeof_type(field.type);
    switch (field.type) {
        case FieldType::F32: {
            float fval;
            auto ival = get_int<int32_t>(ptr);
            std::memcpy(&fval, &ival, sizeof(float));
            return fval;
        }
        case FieldType::F64: {
            double fval;
            auto ival = get_int<int64_t>(ptr);
            std::memcpy(&fval, &ival, sizeof(double));
            return fval;
        }
        case FieldType::I8:
        case FieldType::I16:
        case FieldType::I32:
        case FieldType::I64:
        case FieldType::CHAR:
            return getInteger(src, field, index);
        default:
            throw std::runtime_error("type error");
    }
}

std::string_view StructLayout::getChars(
    const ubyte* src, const Field& field
) const {
    if (field.type != FieldType::CHAR) {
        throw std::runtime_error("'char' field type required");
    }
    auto ptr = reinterpret_cast<const char*>(src + field.offset);
    return std::string_view(ptr, strnlen(ptr, field.elements));
}

dv::value StructLayout::serialize() const {
    auto map = dv::object();
    for (const auto& [name, index] : indices) {
        auto& fieldmap = map.object(name);
        const auto& field = fields[index];
        fieldmap["type"] = to_string(field.type);
        if (field.elements != 1) {
            fieldmap["length"] = field.elements;
        }
        if (field.convertStrategy != FieldConvertStrategy::RESET) {
            fieldmap["convert-strategy"] = to_string(field.convertStrategy);
        }
    }
    return map;
}

void StructLayout::deserialize(const dv::value& src) {
    std::vector<Field> fields;
    for (const auto& [name, fieldmap] : src.asObject()) {
        const auto& typeName = fieldmap["type"].asString();
        FieldType type = FieldType_from_string(typeName);
        
        int elements = 1;
        fieldmap.at("length").get(elements);
        if (elements <= 0) {
            throw std::runtime_error(
                "invalid field " + util::quote(name) + " length: " +
                std::to_string(elements)
            );
        }

        auto convertStrategy = FieldConvertStrategy::RESET;
        if (fieldmap.has("convert-strategy")) {
            convertStrategy = FieldConvertStrategy_from_string(
                fieldmap["convert-strategy"].asString()
            );
        }
        fields.push_back(Field (type, name, elements, convertStrategy));
    }
    *this = create(fields);
}
