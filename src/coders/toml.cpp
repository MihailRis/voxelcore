#include "toml.hpp"

#include <assert.h>
#include <math.h>

#include <iomanip>
#include <sstream>

#include "data/setting.hpp"
#include "io/settings_io.hpp"
#include "util/stringutil.hpp"
#include "BasicParser.hpp"

using namespace toml;

class TomlReader : BasicParser<char> {
    dv::value root;

    // modified version of BasicParser.parseString
    // todo: extract common part
    std::string parseMultilineString() {
        pos += 2;

        std::stringstream ss;
        while (hasNext()) {
            char c = source[pos];
            if (c == '"' && remain() >= 2 && 
                source[pos+1] == '"' && 
                source[pos+2] == '"') {
                pos += 3;
                return ss.str();
            }
            if (c == '\\') {
                pos++;
                c = nextChar();
                if (c >= '0' && c <= '7') {
                    pos--;
                    ss << (char)parseSimpleInt(8);
                    continue;
                }
                switch (c) {
                    case 'n': ss << '\n'; break;
                    case 'r': ss << '\r'; break;
                    case 'b': ss << '\b'; break;
                    case 't': ss << '\t'; break;
                    case 'f': ss << '\f'; break;
                    case '\'': ss << '\\'; break;
                    case '"': ss << '"'; break;
                    case '\\': ss << '\\'; break;
                    case '/': ss << '/'; break;
                    case '\n': continue;
                    default:
                        throw error(
                            "'\\" + std::string({c}) + "' is an illegal escape"
                        );
                }
                continue;
            }
            ss << c;
            pos++;
        }
        throw error("unexpected end");
    }

    dv::value parseValue() {
        char c = peek();
        if (is_digit(c)) {
            int start = pos;
            // parse numeric literal
            auto value = parseNumber(1);
            if (hasNext() && peekNoJump() == '-') {
                while (hasNext()) {
                    c = source[pos];
                    if (!is_digit(c) && c != ':' && c != '.' && c != '-' &&
                        c != 'T' && c != 'Z') {
                        break;
                    }
                    pos++;
                }
                return std::string(source.substr(start, pos - start));
            }
            return value;
        } else if (c == '-' || c == '+') {
            int sign = c == '-' ? -1 : 1;
            pos++;
            // parse numeric literal
            return parseNumber(sign);
        } else if (is_identifier_start(c)) {
            // parse keywords
            std::string keyword = parseName();
            if (keyword == "true" || keyword == "false") {
                return keyword == "true";
            } else if (keyword == "inf") {
                return INFINITY;
            } else if (keyword == "nan") {
                return NAN;
            }
            throw error("unknown keyword " + util::quote(keyword));
        } else if (c == '"' || c == '\'') {
            pos++;
            if (remain() >= 2 && 
                c  == '"' && 
                source[pos] == '"' && 
                source[pos+1] == '"') {
                return parseMultilineString();
            }
            return parseString(c);
        } else if (c == '[') {
            // parse array
            pos++;
            dv::list_t values;
            while (peek() != ']') {
                values.push_back(parseValue());
                if (peek() != ']') {
                    expect(',');
                }
            }
            pos++;
            return dv::value(std::move(values));
        } else if (c == '{') {
            // parse inline table
            pos++;
            auto table = dv::object();
            while (peek() != '}') {
                auto key = parseName();
                expect('=');
                table[key] = parseValue();
                if (peek() != '}') {
                    expect(',');
                }
            }
            pos++;
            return table;
        } else {
            throw error("feature is not supported");
        }
    }

    dv::value& parseLValue(dv::value& root) {
        dv::value* lvalue = &root;
        while (hasNext()) {
            char c = peek();
            std::string name;
            if (c == '\'' || c == '"') {
                pos++;
                name = parseString(c);
            } else {
                name = parseName();
            }
            if (lvalue->getType() == dv::value_type::none) {
                *lvalue = dv::object();
            }
            lvalue = &(*lvalue)[name];
            if (peek() != '.') {
                break;
            }
            pos++;
        }
        return *lvalue;
    }

    void readSection(dv::value& map, dv::value& root) {
        while (hasNext()) {
            skipWhitespace();
            if (!hasNext()) {
                break;
            }
            char c = nextChar();
            if (c == '[') {
                if (hasNext() && peek() == '[') {
                    pos++;
                    // parse list of tables
                    dv::value& list = parseLValue(root);
                    if (list == nullptr) {
                        list = dv::list();
                    } else if (!list.isList()) {
                        throw error("target is not an array");
                    }
                    expect(']');
                    expect(']');
                    dv::value section = dv::object();
                    readSection(section, root);
                    list.add(std::move(section));
                    return;
                }
                // parse table
                dv::value& section = parseLValue(root);
                if (section == nullptr) {
                    section = dv::object();
                } else if (!section.isObject()) {
                    throw error("target is not a table");
                }
                expect(']');
                readSection(section, root);
                return;
            }
            pos--;
            dv::value& lvalue = parseLValue(map);
            expect('=');
            lvalue = parseValue();
            expectNewLine();
        }
    }
public:
    TomlReader(std::string_view file, std::string_view source)
        : BasicParser(file, source), root(dv::object()) {
        hashComment = true;
    }

    dv::value read() {
        skipWhitespace();
        if (!hasNext()) {
            return std::move(root);
        }
        readSection(root, root);
        return std::move(root);
    }
};

void toml::parse(
    SettingsHandler& handler, std::string_view file, std::string_view source
) {
    auto map = parse(file, source);
    
    for (const auto& [sectionName, sectionMap] : map.asObject()) {
        if (!sectionMap.isObject()) {
            continue;
        }
        for (const auto& [name, value] : sectionMap.asObject()) {
            auto fullname = sectionName + "." + name;
            if (handler.has(fullname)) {
                handler.setValue(fullname, value);
            }
        }
    }
}

dv::value toml::parse(std::string_view file, std::string_view source) {
    return TomlReader(file, source).read();
}

static void to_string(std::stringstream& ss, const dv::value& value);

static void list_to_string(std::stringstream& ss, const dv::value& list) {
    ss << "[";
    int index = 0;
    for (const auto& value : list) {
        if (index > 0) {
            ss << ", ";
        }
        to_string(ss, value);
        index++;
    }
    ss << "]";
}

static void object_to_string(std::stringstream& ss, const dv::value& object) {
    ss << "{";
    int index = 0;
    for (const auto& [key, value] : object.asObject()) {
        if (index > 0) {
            ss << ", ";
        }
        ss << key << " = ";
        to_string(ss, value);
        index++;
    }
    ss << "}";
}

static void to_string(std::stringstream& ss, const dv::value& value) {
    if (value.isObject()) {
        object_to_string(ss, value);
    } else if (value.isList()) {
        list_to_string(ss, value);
    } else {
        ss << value;
    }
}

std::string toml::stringify(const dv::value& root, const std::string& name) {
    std::stringstream ss;
    if (!name.empty()) {
        ss << "[" << name << "]\n";
    }
    for (const auto& [key, value] : root.asObject()) {
        if (!value.isObject() && !value.isList()) {
            ss << key << " = " << value << "\n";
        }
    }
    for (const auto& [key, value] : root.asObject()) {
        if (value.isObject()) {
            ss << "\n" << toml::stringify(value,
                      name.empty() ? key : name + "." + key);
        } else if (value.isList()) {
            ss << (name.empty() ? key : name + "." + key) << " = [";
            for (size_t i = 0; i < value.size(); i++) {
                if (i > 0) {
                    ss << ", ";
                }
                to_string(ss, value[i]);
            }
            ss << "]";
        }
    }
    return ss.str();
}

std::string toml::stringify(SettingsHandler& handler) {
    auto& sections = handler.getSections();

    std::stringstream ss;
    for (auto& section : sections) {
        ss << "[" << section.name << "]\n";
        for (const std::string& key : section.keys) {
            ss << key << " = ";
            auto setting = handler.getSetting(section.name + "." + key);
            assert(setting != nullptr);
            if (auto integer = dynamic_cast<IntegerSetting*>(setting)) {
                ss << integer->get();
            } else if (auto number = dynamic_cast<NumberSetting*>(setting)) {
                ss << number->get();
            } else if (auto flag = dynamic_cast<FlagSetting*>(setting)) {
                ss << (flag->get() ? "true" : "false");
            } else if (auto string = dynamic_cast<StringSetting*>(setting)) {
                ss << util::escape(string->get());
            }
            ss << "\n";
        }
        ss << "\n";
    }
    return ss.str();
}
