#pragma once

#ifdef VC_ENABLE_REFLECTION
#define VC_ENUM_METADATA(NAME) static inline util::EnumMetadata<NAME> NAME##Meta {
#define VC_ENUM_END };

#include <map>
#include <string>
#include <utility>

namespace util {
    template<typename EnumT>
    class EnumMetadata {
    public:
        EnumMetadata(
            std::initializer_list<std::pair<const std::string_view, EnumT>> items
        )
            : items(items) {
            for (const auto& [name, item] : items) {
                names[item] = name;
            }
        }

        std::string_view getName(EnumT item) const {
            const auto& found = names.find(item);
            if (found == names.end()) {
                return "";
            }
            return found->second;
        }

        std::string getNameString(EnumT item) const {
            return std::string(getName(item));
        }

        bool getItem(std::string_view name, EnumT& dst) const {
            const auto& found = items.find(name);
            if (found == items.end()) {
                return false;
            }
            dst = found->second;
            return true;
        }

        size_t size() const {
            return items.size();
        }
    private:
        std::map<std::string_view, EnumT> items;
        std::map<EnumT, std::string_view> names;
    };
}

#else
#include <initializer_list>

#define VC_ENUM_METADATA(NAME) \
    struct NAME##__PAIR {const char* n; NAME i;}; \
    [[maybe_unused]] static inline std::initializer_list<NAME##__PAIR> NAME##_PAIRS {
#define VC_ENUM_END };
#endif // VC_ENABLE_REFLECTION
