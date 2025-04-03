#pragma once

#include <string>
#include <vector>
#include <memory>
#include <unordered_map>

#include "io/fwd.hpp"

namespace langs {
    const char LANG_FILE_EXT[] = ".txt";
    const char TEXTS_FOLDER[] = "texts";
    const char FALLBACK_DEFAULT[] = "en_US";

    /*
        Translation is mostly used for rendered text,
        so Font.draw and Lang both use wstring for strings.
        
        Translation key is wstring too, allowing to use it as value 
        if no any translation found, without conversion required

        Key syntax: `modid:context.key` where modid is added at runtime for
        content pack texts
     */
    class Lang {
        std::string locale;
        std::unordered_map<std::wstring, std::wstring> map;
    public:
        Lang(std::string locale);

        const std::wstring& get(const std::wstring& key) const;
        void put(const std::wstring& key, const std::wstring& text);

        const std::string& getId() const;
    };

    struct LocaleInfo {
        std::string locale;
        std::string name;
    };

    std::string locale_by_envlocale(const std::string& envlocale);

    const std::string& get_current();
    const std::unordered_map<std::string, LocaleInfo>& get_locales_info();

    const std::wstring& get(const std::wstring& key);
    const std::wstring& get(
        const std::wstring& key, const std::wstring& context
    );

    /// @brief Change locale to specified
    /// @param locale isolanguage_ISOCOUNTRY (example: en_US)
    void setup(std::string locale, const std::vector<io::path>& roots);
}
