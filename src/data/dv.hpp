#pragma once

#include <stdint.h>
#include <string>
#include <memory>
#include <iosfwd>
#include <vector>
#include <cstring>
#include <stdexcept>
#include <unordered_map>

#ifdef VC_ENABLE_REFLECTION
#include "util/EnumMetadata.hpp"
#endif

namespace util {
    template<class T> class Buffer;
}

namespace dv {
    using integer_t = int64_t;
    using number_t = double;
    using boolean_t = bool;
    using byte_t = unsigned char;
    using key_t = std::string;

    enum class value_type : uint8_t {
        none = 0,
        number,
        boolean,
        integer,
        object,
        list,
        bytes,
        string
    };

    inline const std::string& type_name(value_type type) {
        static std::string type_names[] = {
            "none",
            "number",
            "boolean",
            "integer",
            "object",
            "list",
            "bytes",
            "string"
        };
        return type_names[static_cast<int>(type)];
    }

    class value;

    using list_t = std::vector<value>;
    using map_t = std::unordered_map<key_t, value>;
    using pair = std::pair<const key_t, value>;

    using reference = value&;
    using const_reference = const value&;

    namespace objects {
        using Object = std::unordered_map<key_t, value>;
        using List = std::vector<value>;
        using Bytes = util::Buffer<byte_t>;
    }

    /// @brief nullable value reference returned by value.at(...)
    struct optionalvalue {
        value* ptr;

        optionalvalue(value* ptr) noexcept : ptr(ptr) {}

        inline operator bool() const noexcept {
            return ptr != nullptr;
        }

        inline value* operator->() noexcept {
            return ptr;
        }

        inline const value* operator->() const noexcept {
            return ptr;
        }

        inline value& operator*() noexcept {
            return *ptr;
        }

        inline const value& operator*() const noexcept {
            return *ptr;
        }

#ifdef VC_ENABLE_REFLECTION
        template <typename T>
        bool get(T& dst, const util::EnumMetadata<T>& mt) const;
#endif

        bool get(std::string& dst) const;
        bool get(bool& dst) const;
        bool get(char& dst) const;
        bool get(short& dst) const;
        bool get(int& dst) const;
        bool get(long& dst) const;
        bool get(long long& dst) const;
        bool get(unsigned char& dst) const;
        bool get(unsigned short& dst) const;
        bool get(unsigned int& dst) const;
        bool get(unsigned long& dst) const;
        bool get(unsigned long long& dst) const;
        bool get(float& dst) const;
        bool get(double& dst) const;
    };

    inline void throw_type_error(value_type got, value_type expected) {
        // place breakpoint here to find cause
        throw std::runtime_error(
            "type error: expected " + type_name(expected) + ", got " +
            type_name(got)
        );
    }

    inline void check_type(value_type got, value_type expected) {
        if (got != expected) {
            throw_type_error(got, expected);
        }
    }

    class value {
        value_type type = value_type::none;
        union value_u {
            integer_t integer;
            number_t number;
            boolean_t boolean;
            std::unique_ptr<std::string> string;
            std::shared_ptr<objects::Object> object;
            std::shared_ptr<objects::List> list;
            std::shared_ptr<objects::Bytes> bytes;
            value_u() noexcept {}
            ~value_u() noexcept {}
        } val;

        inline value& setBoolean(boolean_t v) noexcept {
            this->~value();
            type = value_type::boolean;
            val.boolean = v;
            return *this;
        }
        inline value& setInteger(integer_t v) noexcept {
            this->~value();
            type = value_type::integer;
            val.integer = v;
            return *this;
        }
        inline value& setNumber(number_t v) noexcept {
            this->~value();
            type = value_type::number;
            val.number = v;
            return *this;
        }
        inline value& setNone() noexcept {
            this->~value();
            type = value_type::none;
            return *this;
        }
        inline value& setString(std::string v) noexcept {
            this->~value();
            new(&val.string)std::unique_ptr<std::string>(
                std::make_unique<std::string>(std::move(v)));
            type = value_type::string;
            return *this;
        }
        inline value& setString(std::unique_ptr<std::string> v) noexcept {
            this->~value();
            new(&val.string)std::unique_ptr<std::string>(std::move(v));
            type = value_type::string;
            return *this;
        }
        inline value& setList(std::shared_ptr<objects::List> ptr) noexcept {
            this->~value();
            new(&val.list)std::shared_ptr<objects::List>(std::move(ptr));
            type = value_type::list;
            return *this;
        }
        inline value& setObject(std::shared_ptr<objects::Object> ptr) noexcept {
            this->~value();
            new(&val.object)std::shared_ptr<objects::Object>(std::move(ptr));
            type = value_type::object;
            return *this;
        }
        inline value& setBytes(std::shared_ptr<objects::Bytes> ptr) noexcept {
            this->~value();
            new(&val.bytes)std::shared_ptr<objects::Bytes>(std::move(ptr));
            type = value_type::bytes;
            return *this;
        }
    public:
        value() noexcept : type(value_type::none) {}
        
        /// @brief Constructor for fundamental types
        template<typename T>
        value(T v, std::enable_if_t<std::is_fundamental<T>::value, int> = 0) noexcept {
            this->operator=(v);
        }
        value(std::string v) noexcept {
            this->operator=(std::move(v));
        }
        value(std::shared_ptr<objects::Object> v) noexcept {
            this->operator=(std::move(v));
        }
        value(std::shared_ptr<objects::List> v) noexcept {
            this->operator=(std::move(v));
        }
        value(std::shared_ptr<objects::Bytes> v) noexcept {
            this->operator=(std::move(v));
        }
        value(list_t values) {
            this->operator=(std::make_shared<list_t>(std::move(values)));
        }

        value(const value& v) noexcept : type(value_type::none) {
            this->operator=(v);
        }

        value(value&& v) noexcept {
            this->operator=(std::move(v));
        }

        ~value() noexcept {
            switch (type) {
                case value_type::object:
                    val.object.reset();
                    break;
                case value_type::list:
                    val.list.reset();
                    break;
                case value_type::bytes:
                    val.bytes.reset();
                    break;
                case value_type::string:
                    val.string.reset();
                    break;
                default:
                    break;
            }
        }

        inline value& operator=(std::nullptr_t) {
            return setNone();
        }

        template<typename T>
        inline std::enable_if_t<std::is_integral<T>() && !std::is_same<T, bool>(), value&>
        operator=(T v) {
            return setInteger(v);
        }

        inline value& operator=(float v) {
            return setNumber(v);
        }
        inline value& operator=(double v) {
            return setNumber(v);
        }
        inline value& operator=(bool v) {
            return setBoolean(v);
        }
        inline value& operator=(std::string_view v) {
            return setString(std::string(v));
        }
        inline value& operator=(std::string v) {
            return setString(std::move(v));
        }
        inline value& operator=(const char* v) {
            return setString(v);
        }
        inline value& operator=(std::shared_ptr<objects::List> ptr) {
            return setList(std::move(ptr));
        }
        inline value& operator=(std::shared_ptr<objects::Object> ptr) {
            return setObject(std::move(ptr));
        }
        inline value& operator=(std::shared_ptr<objects::Bytes> ptr) {
            return setBytes(std::move(ptr));
        }
        value& operator=(const objects::Bytes& bytes);

        inline value& operator=(const value& v) {
            switch (v.type) {
                case value_type::object:
                    setObject(v.val.object);
                    break;
                case value_type::list:
                    setList(v.val.list);
                    break;
                case value_type::bytes:
                    setBytes(v.val.bytes);
                    break;
                case value_type::string:
                    setString(*v.val.string);
                    break;
                case value_type::boolean:
                    setBoolean(v.val.boolean);
                    break;
                case value_type::integer:
                    setInteger(v.val.integer);
                    break;
                case value_type::number:
                    setNumber(v.val.number);
                    break;
                case value_type::none:
                    setNone();
                    break;
            }
            return *this;
        }

        inline value& operator=(value&& v) noexcept {
            if (type < value_type::object) {
                type = v.type;
                switch (v.type) {
                    case value_type::none:
                        break;
                    case value_type::integer:
                        val.integer = v.val.integer;
                        break;
                    case value_type::number:
                        val.number = v.val.number;
                        break;
                    case value_type::boolean:
                        val.boolean = v.val.boolean;
                        break;
                    case value_type::string:
                        new(&val.string)std::unique_ptr<std::string>(
                            std::move(v.val.string));
                        break;
                    case value_type::object:
                        new(&val.object)std::shared_ptr<objects::Object>(
                            std::move(v.val.object));
                        break;
                    case value_type::list:
                        new(&val.list)std::shared_ptr<objects::List>(
                            std::move(v.val.list));
                        break;
                    case value_type::bytes:
                        new(&val.list)std::shared_ptr<objects::Bytes>(
                            std::move(v.val.bytes));
                        break;
                }
            } else {
                switch (v.type) {
                    case value_type::object:
                        setObject(std::move(v.val.object));
                        break;
                    case value_type::list:
                        setList(std::move(v.val.list));
                        break;
                    case value_type::bytes:
                        setBytes(std::move(v.val.bytes));
                        break;
                    case value_type::string:
                        setString(std::move(v.val.string));
                        break;
                    case value_type::boolean:
                        setBoolean(v.val.boolean);
                        break;
                    case value_type::integer:
                        setInteger(v.val.integer);
                        break;
                    case value_type::number:
                        setNumber(v.val.number);
                        break;
                    case value_type::none:
                        setNone();
                        break;
                }
            }
            return *this;
        }

        void add(value v);

        template<class T>
        inline void add(T v) {
            return add(value(v));
        }

        void erase(const key_t& key);

        void erase(size_t index);

        value& operator[](const key_t& key);

        const value& operator[](const key_t& key) const;

        void merge(dv::value&& other, bool deep);

        value& operator[](size_t index);

        const value& operator[](size_t index) const;

        bool operator!=(std::nullptr_t) const noexcept {
            return type != value_type::none;
        }

        bool operator==(std::nullptr_t) const noexcept {
            return type == value_type::none;
        }

        value& object(const key_t& key);
        
        value& list(const key_t& key);

        value& object();
        
        value& list();

        list_t::iterator begin();
        list_t::iterator end();

        list_t::const_iterator begin() const;
        list_t::const_iterator end() const;

        const std::string& asString() const;

        integer_t asInteger() const;

        number_t asNumber() const;

        boolean_t asBoolean() const;

        objects::Bytes& asBytes();

        const objects::Bytes& asBytes() const;   

        const objects::Object& asObject() const;

        inline value_type getType() const {
            return type;
        }

        std::string asString(std::string def) const {
            if (type != value_type::string) {
                return def;
            }
            return *val.string;
        }

        std::string asString(const char* s) const {
            return asString(std::string(s));
        }

        integer_t asBoolean(boolean_t def) const {
            switch (type) {
                case value_type::boolean: 
                    return val.boolean;
                default:
                    return def;
            }
        }

        integer_t asInteger(integer_t def) const {
            switch (type) {
                case value_type::integer: 
                    return val.integer;
                case value_type::number: 
                    return static_cast<integer_t>(val.number);
                default:
                    return def;
            }
        }

        integer_t asNumber(integer_t def) const {
            switch (type) {
                case value_type::integer: 
                    return static_cast<number_t>(val.integer);
                case value_type::number: 
                    return val.number;
                default:
                    return def;
            }
        }

        optionalvalue at(const key_t& k) const {
            check_type(type, value_type::object);
            const auto& found = val.object->find(k);
            if (found == val.object->end()) {
                return optionalvalue(nullptr);
            }
            return optionalvalue(&found->second);
        }

        optionalvalue at(size_t index) {
            check_type(type, value_type::list);
            return optionalvalue(&val.list->at(index));
        }

        const optionalvalue at(size_t index) const {
            check_type(type, value_type::list);
            return optionalvalue(&val.list->at(index));
        }

        bool has(const key_t& k) const;

        size_t size() const noexcept;

        size_t length() const noexcept {
            return size();
        }
        inline bool empty() const noexcept {
            return size() == 0;
        }

        inline bool isString() const noexcept {
            return type == value_type::string;
        }
        inline bool isObject() const noexcept {
            return type == value_type::object;
        }
        inline bool isList() const noexcept {
            return type == value_type::list;
        }
        inline bool isInteger() const noexcept {
            return type == value_type::integer;
        }
        inline bool isNumber() const noexcept {
            return type == value_type::number;
        }
        inline bool isBoolean() const noexcept {
            return type == value_type::boolean;
        }
    };

    inline bool is_numeric(const value& val) {
        return val.isInteger() || val.isNumber();
    }
}

namespace dv {
    inline const std::string& type_name(const value& value) {
        return type_name(value.getType());
    }

    inline value object() {
        return std::make_shared<objects::Object>();
    }

    inline value object(std::initializer_list<pair> pairs) {
        return std::make_shared<objects::Object>(std::move(pairs));
    }

    inline value list() {
        return std::make_shared<objects::List>();
    }

    inline value list(std::initializer_list<value> values) {
        return std::make_shared<objects::List>(std::move(values));
    }

    template<typename T> inline bool get_to_int(value* ptr, T& dst) {
        if (ptr) {
            dst = ptr->asInteger();
            return true;
        }
        return false;
    }
    template<typename T> inline bool get_to_num(value* ptr, T& dst) {
        if (ptr) {
            dst = ptr->asNumber();
            return true;
        }
        return false;
    }

#ifdef VC_ENABLE_REFLECTION
    template <typename T>
    inline bool optionalvalue::get(T& dst, const util::EnumMetadata<T>& mt) const {
        if (ptr) {
            return mt.getItem(ptr->asString(), dst);
        }
        return false;
    }
#endif

    inline bool optionalvalue::get(std::string& dst) const {
        if (ptr) {
            dst = ptr->asString();
            return true;
        }
        return false;
    }

    inline bool optionalvalue::get(bool& dst) const {
        if (ptr) {
            dst = ptr->asBoolean();
            return true;
        }
        return false;
    }

    inline bool optionalvalue::get(char& dst) const {
        return get_to_int(ptr, dst);
    }
    inline bool optionalvalue::get(short& dst) const {
        return get_to_int(ptr, dst);
    }
    inline bool optionalvalue::get(int& dst) const {
        return get_to_int(ptr, dst);
    }
    inline bool optionalvalue::get(long& dst) const {
        return get_to_int(ptr, dst);
    }
    inline bool optionalvalue::get(long long& dst) const {
        return get_to_int(ptr, dst);
    }
    inline bool optionalvalue::get(unsigned char& dst) const {
        return get_to_int(ptr, dst);
    }
    inline bool optionalvalue::get(unsigned short& dst) const {
        return get_to_int(ptr, dst);
    }
    inline bool optionalvalue::get(unsigned int& dst) const {
        return get_to_int(ptr, dst);
    }
    inline bool optionalvalue::get(unsigned long& dst) const {
        return get_to_int(ptr, dst);
    }
    inline bool optionalvalue::get(unsigned long long& dst) const {
        return get_to_int(ptr, dst);
    }
    inline bool optionalvalue::get(float& dst) const {
        return get_to_num(ptr, dst);
    }
    inline bool optionalvalue::get(double& dst) const {
        return get_to_num(ptr, dst);
    }
}

std::ostream& operator<<(std::ostream& stream, const dv::value& value);
