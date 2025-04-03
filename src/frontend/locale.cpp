#include "locale.hpp"

#include <utility>

#include "coders/json.hpp"
#include "coders/BasicParser.hpp"
#include "io/io.hpp"
#include "util/stringutil.hpp"
#include "data/dv.hpp"
#include "debug/Logger.hpp"

static debug::Logger logger("locale");

namespace fs = std::filesystem;
using namespace std::literals;
using namespace langs;

namespace {
    std::unique_ptr<langs::Lang> current;
    std::unordered_map<std::string, LocaleInfo> locales_info;
}

langs::Lang::Lang(std::string locale) : locale(std::move(locale)) {
}

const std::wstring& langs::Lang::get(const std::wstring& key) const  {
    auto found = map.find(key);
    if (found == map.end()) {
        return key;
    }
    return found->second;
}

void langs::Lang::put(const std::wstring& key, const std::wstring& text) {
    map[key] = text;
}

const std::string& langs::Lang::getId() const {
    return locale;
}

/// @brief Language key-value txt files parser
namespace {
    class Reader : BasicParser<char> {
    public:
        Reader(std::string_view file, std::string_view source)
            : BasicParser(file, source) {
            hashComment = true;
        }

        void read(langs::Lang& lang, const std::string &prefix) {
            skipWhitespace();
            while (hasNext()) {
                std::string key = parseString('=', true);
                util::trim(key);
                key = prefix + key;
                std::string text = parseString('\n', false);
                util::trim(text);
                lang.put(util::str2wstr_utf8(key), util::str2wstr_utf8(text));
                skipWhitespace();
            }
        }
    };
}

static void load_locales_info(std::string& fallback) {
    auto file = io::path("res:") / langs::TEXTS_FOLDER / "langs.json";
    auto root = io::read_json(file);

    ::locales_info.clear();
    root.at("fallback").get(fallback);

    if (auto found = root.at("langs")) {
        auto& langs = *found;
        auto logline = logger.info();
        logline << "locales ";
        for (const auto& [key, langInfo] : langs.asObject()) {
            std::string name;
            if (langInfo.isObject()) {
                name = langInfo["name"].asString("none");
            } else {
                continue;
            }
            logline << key << " ";
            ::locales_info[key] = LocaleInfo {key, name};
        } 
        logline << "added";
    }
}

static void load(
    const std::string& locale,
    const std::vector<io::path>& roots,
    Lang& lang
) {
    io::path filename = io::path(TEXTS_FOLDER) / (locale + LANG_FILE_EXT);
    io::path core_file = io::path("res:") / filename;

    if (io::is_regular_file(core_file)) {
        std::string text = io::read_string(core_file);
        Reader reader(core_file.string(), text);
        reader.read(lang, "");
    }
    for (auto root : roots) {
        io::path file = root / filename;
        if (io::is_regular_file(file)) {
            std::string text = io::read_string(file);
            Reader reader(file.string(), text);
            reader.read(lang, "");
        }
    }
}
static void load(
    const std::string& locale,
    const std::string& fallback,
    const std::vector<io::path>& roots
) {
    auto lang = std::make_unique<Lang>(locale);
    load(fallback, roots, *lang.get());
    if (locale != fallback) {
        load(locale, roots, *lang.get());
    }
    current = std::move(lang);
}

const std::string& langs::get_current() {
    if (current == nullptr) {
        throw std::runtime_error("localization is not initialized");
    }
    return current->getId();
}

const std::unordered_map<std::string, LocaleInfo>& langs::get_locales_info() {
    return ::locales_info;
}

std::string langs::locale_by_envlocale(const std::string& envlocale){
    std::string fallback = FALLBACK_DEFAULT;
    if (locales_info.size() == 0) {
        load_locales_info(fallback);
    }
    if (locales_info.find(envlocale) != locales_info.end()) {
        logger.info() << "locale " << envlocale << " is automatically selected";
        return envlocale;
    }
    else {
        for (const auto& loc : locales_info) {
            if (loc.first.find(envlocale.substr(0, 2)) != std::string::npos) {
                logger.info() << "locale " << loc.first << " is automatically selected";
                return loc.first;
            }
        }
        logger.info() << "locale " << fallback << " is automatically selected";
        return fallback;
    }
}

void langs::setup(
    std::string locale,
    const std::vector<io::path>& roots
) {
    std::string fallback = langs::FALLBACK_DEFAULT;
    load_locales_info(fallback);
    if (locales_info.find(locale) == locales_info.end()) {
        locale = fallback;
    }
    load(locale, fallback, roots);
}

const std::wstring& langs::get(const std::wstring& key) {
    return current->get(key);
}

const std::wstring& langs::get(const std::wstring& key, const std::wstring& context) {
    std::wstring ctxkey = context + L"." + key;
    const std::wstring& text = current->get(ctxkey);
    if (&ctxkey != &text) {
        return text;
    }
    return current->get(key);
}
