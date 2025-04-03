#include "json.hpp"

#include <math.h>

#include <iomanip>
#include <memory>
#include <sstream>

#include "util/stringutil.hpp"
#include "BasicParser.hpp"

using namespace json;

namespace {
    class Parser : BasicParser<char> {
        public:
        Parser(std::string_view filename, std::string_view source);

        dv::value parse();
    private:
        dv::value parseList();
        dv::value parseObject();
        dv::value parseValue();
    };
}

inline void newline(
    std::stringstream& ss, bool nice, uint indent, const std::string& indentstr
) {
    if (nice) {
        ss << "\n";
        for (uint i = 0; i < indent; i++) {
            ss << indentstr;
        }
    } else {
        ss << ' ';
    }
}

void stringifyObj(
    const dv::value& obj,
    std::stringstream& ss,
    int indent,
    const std::string& indentstr,
    bool nice,
    bool escapeUtf8
);

void stringifyList(
    const dv::value& list,
    std::stringstream& ss,
    int indent,
    const std::string& indentstr,
    bool nice,
    bool escapeUtf8
);

void stringifyValue(
    const dv::value& value,
    std::stringstream& ss,
    int indent,
    const std::string& indentstr,
    bool nice,
    bool escapeUtf8
) {
    using dv::value_type;

    switch (value.getType()) {
        case value_type::object:
            stringifyObj(value, ss, indent, indentstr, nice, escapeUtf8);
            break;
        case value_type::list:
            stringifyList(value, ss, indent, indentstr, nice, escapeUtf8);
            break;
        case value_type::bytes: {
            const auto& bytes = value.asBytes();
            ss << "\"" << util::base64_encode(bytes.data(), bytes.size());
            ss << "\"";
            break;
        }
        case value_type::string:
            ss << util::escape(value.asString(), escapeUtf8);
            break;
        case value_type::number:
            ss << std::setprecision(15) << value.asNumber();
            break;
        case value_type::integer:
            ss << value.asInteger();
            break;
        case value_type::boolean:
            ss << (value.asBoolean() ? "true" : "false");
            break;
        case value_type::none:
            ss << "null";
            break; 
    }
}

void stringifyList(
    const dv::value& list,
    std::stringstream& ss,
    int indent,
    const std::string& indentstr,
    bool nice,
    bool escapeUtf8
) {
    if (list.empty()) {
        ss << "[]";
        return;
    }
    ss << "[";
    for (size_t i = 0; i < list.size(); i++) {
        if (i > 0 || nice) {
            newline(ss, nice, indent, indentstr);
        }
        const auto& value = list[i];
        stringifyValue(value, ss, indent + 1, indentstr, nice, escapeUtf8);
        if (i + 1 < list.size()) {
            ss << ',';
        }
    }
    if (nice) {
        newline(ss, true, indent - 1, indentstr);
    }
    ss << ']';
}

void stringifyObj(
    const dv::value& obj,
    std::stringstream& ss,
    int indent,
    const std::string& indentstr,
    bool nice,
    bool escapeUtf8
) {
    if (obj.empty()) {
        ss << "{}";
        return;
    }
    ss << "{";
    size_t index = 0;
    for (auto& [key, value] : obj.asObject()) {
        if (index > 0 || nice) {
            newline(ss, nice, indent, indentstr);
        }
        ss << util::escape(key) << ": ";
        stringifyValue(value, ss, indent + 1, indentstr, nice, escapeUtf8);
        index++;
        if (index < obj.size()) {
            ss << ',';
        }
    }
    if (nice) {
        newline(ss, true, indent - 1, indentstr);
    }
    ss << '}';
}

std::string json::stringify(
    const dv::value& value,
    bool nice,
    const std::string& indent,
    bool escapeUtf8
) {
    std::stringstream ss;
    stringifyValue(value, ss, 1, indent, nice, escapeUtf8);
    return ss.str();
}

Parser::Parser(std::string_view filename, std::string_view source)
    : BasicParser(filename, source) {
}

dv::value Parser::parse() {
    char next = peek();
    if (next == '{') {
        return parseObject();
    } else if (next == '[') {
        return parseList();
    }
    throw error("'{' or '[' expected");
}

dv::value Parser::parseObject() {
    expect('{');
    auto object = dv::object();
    while (peek() != '}') {
        if (peek() == '#') {
            skipLine();
            continue;
        }
        expect('"');
        std::string key = parseString('"');
        char next = peek();
        if (next != ':') {
            throw error("':' expected");
        }
        pos++;
        object[key] = parseValue();
        next = peek();
        if (next == ',') {
            pos++;
        } else if (next == '}') {
            break;
        } else {
            throw error("',' expected");
        }
    }
    pos++;
    return object;
}

dv::value Parser::parseList() {
    expect('[');
    auto list = dv::list();
    while (peek() != ']') {
        if (peek() == '#') {
            skipLine();
            continue;
        }
        list.add(parseValue());

        char next = peek();
        if (next == ',') {
            pos++;
        } else if (next == ']') {
            break;
        } else {
            throw error("',' expected");
        }
    }
    pos++;
    return list;
}

dv::value Parser::parseValue() {
    char next = peek();
    if (next == '-' || next == '+' || is_digit(next)) {
        auto numeric = parseNumber();
        if (numeric.isInteger()) {
            return numeric.asInteger();
        }
        return numeric.asNumber();
    }
    if (is_identifier_start(next)) {
        std::string literal = parseName();
        if (literal == "true") {
            return true;
        } else if (literal == "false") {
            return false;
        } else if (literal == "inf") {
            return INFINITY;
        } else if (literal == "nan") {
            return NAN;
        } else if (literal == "null") {
            return nullptr;
        }
        throw error("invalid keyword " + literal);
    }
    if (next == '{') {
        return parseObject();
    }
    if (next == '[') {
        return parseList();
    }
    if (next == '"' || next == '\'') {
        pos++;
        return parseString(next);
    }
    throw error("unexpected character '" + std::string({next}) + "'");
}

dv::value json::parse(
    std::string_view filename, std::string_view source
) {
    Parser parser(filename, source);
    return parser.parse();
}

dv::value json::parse(std::string_view source) {
    return parse("[string]", source);
}
