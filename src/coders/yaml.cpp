#include "yaml.hpp"
#include "BasicParser.hpp"

#include <iomanip>

using namespace yaml;

namespace {
    enum Chomping {
        CLIP, STRIP, KEEP
    };

    class Parser : BasicParser<char> {
    public:
        Parser(std::string_view filename, std::string_view source);

        dv::value parseValue();
        dv::value parseFullValue(int indent);
        dv::value parseArray(int indent = 0);
        dv::value parseObject(dv::value&& object, int indent = 0);
        dv::value parseInlineArray();
        dv::value parseInlineObject();
    private:
        int countIndent();
        bool expectIndent(int indent);
        std::string_view readYamlIdentifier();
        std::string readMultilineString(int indent, bool eols, Chomping chomp);
    };
}

inline bool is_yaml_identifier_char(int c) {
    return c > 20 && c != ':' && c != ' ' && c != '\n' && c != '\r' &&
           c != '\t' && c != '\f' && c != '\v';
}

static dv::value perform_literal(std::string_view literal) {
    if (literal == "true" || literal == "True" || 
        literal == "false" || literal == "False") {
        return literal[0] == 't';
    }
    if (literal == "null" || literal == "Null") {
        return nullptr;
    }
    return std::string(literal);
}

Parser::Parser(std::string_view filename, std::string_view source)
    : BasicParser(filename, source) {
    hashComment = true;
}

bool Parser::expectIndent(int required) {
    int indent = 0;
    while (hasNext() && source[pos] == ' ' && indent < required) {
        indent++;
        pos++;
    }
    return indent >= required;
}

std::string Parser::readMultilineString(int indent, bool eols, Chomping chomp) {
    int next_indent = countIndent();
    if (next_indent <= indent) {
        throw error("indentation error");
    }
    std::stringstream ss;
    ss << readUntilEOL();
    if (hasNext()) {
        skip(1);
    }
    int trailingEmpties = 0;
    while (true) {
        while (expectIndent(next_indent)) {
            trailingEmpties = 0;
            ss << (eols ? '\n' : ' ');
            ss << readUntilEOL();
            if (hasNext()) {
                skip(1);
            }
        }
        while (true) {
            skipWhitespace(false);
            if (!hasNext() || source[pos] != '\n') {
                break;
            }
            skip(1);
            trailingEmpties++;
        }
        if (!expectIndent(next_indent)) {
            break;
        }
        pos = linestart;
    }
    if (chomp == KEEP) {
        for (int i = 0; i < trailingEmpties - 1; i++) {
            ss << (eols ? '\n' : ' ');
        }
    }
    ss << '\n';

    pos = linestart;

    auto string = ss.str();
    if (chomp == STRIP) {
        util::trim(string);
    }
    return string;
}

std::string_view Parser::readYamlIdentifier() {
    char c = peek();
    if (!is_yaml_identifier_char(c)) {
        throw error("identifier expected");
    }
    int start = pos;
    while (hasNext() && is_yaml_identifier_char(source[pos])) {
        pos++;
    }
    return source.substr(start, pos - start);
}

int Parser::countIndent() {
    int indent = 0;
    while (hasNext() && source[pos] == ' ') {
        indent++;
        pos++;
    }
    return indent;
}

dv::value Parser::parseValue() {
    char c = peek();
    if (is_digit(c)) {
        return parseNumber(1);
    } else if (c == '-' || c == '+') {
        skip(1);
        return parseNumber(c == '-' ? -1 : 1);
    } else if (c == '"' || c == '\'') {
        skip(1);
        return parseString(c, true);
    } else if (c == '[') {
        return parseInlineArray();
    } else if (c == '{') {
        return parseInlineObject();
    } else {
        return perform_literal(readUntilEOL());
    }
    throw error("unexpected character");
}

dv::value Parser::parseInlineArray() {
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

dv::value Parser::parseInlineObject() {
    expect('{');
    dv::value object = dv::object();
    while (peek() != '}') {
        if (peek() == '#') {
            skipLine();
            continue;
        }
        auto name = readYamlIdentifier();
        expect(':');
        object[std::string(name)] = parseValue();

        char next = peek();
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

dv::value Parser::parseFullValue(int indent) {
    dv::value value;
    char c = source[pos];
    if (c == '\n') {
        skip(1);
        skipEmptyLines();
        int init_pos = pos;
        int next_indent = countIndent();
        if (next_indent < indent) {
            throw error("indentation error");
        }
        if (source[pos] == '-') {
            pos = init_pos;
            return parseArray(next_indent);
        } else {
            pos = init_pos;
            return parseObject(dv::object(), next_indent);
        }
    } else if (is_digit(c)) {
        return parseNumber(1);
    } else if (c == '-' || c == '+') {
        skip(1);
        return parseNumber(c == '-' ? -1 : 1);
    } else if (c == '"' || c == '\'') {
        skip(1);
        return parseString(c, true);
    } else if (c == '[') {
        return parseInlineArray();
    } else if (c == '{') {
        return parseInlineObject();
    } else if (c == '|' || c == '>') {
        skip(1);
        Chomping chomp = CLIP;
        if (source[pos] == '-' || source[pos] == '+') {
            chomp = source[pos] == '-' ? STRIP : KEEP;
            skip(1);
        }
        skipWhitespace(false);
        expectNewLine();
        return readMultilineString(indent, c == '|', chomp);
    } else {
        return perform_literal(readUntilEOL());
    }
}

dv::value Parser::parseArray(int indent) {
    dv::value list = dv::list();

    while (hasNext()) {
        skipEmptyLines();
        int next_indent = countIndent();
        if (next_indent < indent) {
            pos = linestart;
            break;
        }
        expect('-');
        skipWhitespace();
        size_t nlpos = source.find('\n', pos);
        size_t colonpos = source.find(':', pos);
        if (nlpos == std::string::npos && colonpos == std::string::npos) {
            list.add(perform_literal(readUntilEOL()));
            break;
        }
        if (nlpos < colonpos) {
            list.add(parseFullValue(next_indent));
            skipLine();
        } else {
            auto name = readYamlIdentifier();
            expect(':');
            skipWhitespace(false);
            dv::value object = dv::object();
            object[std::string(name)] = parseFullValue(next_indent);
            skipEmptyLines();
            next_indent = countIndent();
            if (next_indent > indent) {
                pos = linestart;
                object = parseObject(std::move(object), next_indent);
            } else {
                pos = linestart;
            }
            list.add(std::move(object));
        }
    }
    return list;
}

dv::value Parser::parseObject(dv::value&& object, int indent) {
    skipEmptyLines();
    while (hasNext()) {
        size_t prev_pos = pos;
        int next_indent = countIndent();
        if (source[pos] == '\n') {
            skip(1);
            continue;
        }
        if (next_indent < indent) {
            pos = prev_pos;
            break;
        }
        char c = peek();
        if (!is_yaml_identifier_char(c)) {
            if (!is_whitespace(c)) {
                throw error("invalid character");
            }
            continue;
        }
        auto name = readYamlIdentifier();
        expect(':');
        skipWhitespace(false);
        object[std::string(name)] = parseFullValue(indent);
        skipEmptyLines();
    }
    return std::move(object);
}

dv::value yaml::parse(std::string_view filename, std::string_view source) {
    return Parser(filename, source).parseObject(dv::object());
}

dv::value yaml::parse(std::string_view source) {
    return parse("[string]", source);
}

static void add_indent(std::stringstream& ss, int indent) {
    for (int i = 0; i < indent; i++) {
        ss << "  ";
    }
}

static void insert_string(
    std::stringstream& ss, const std::string& string, int indent
) {
    bool has_spec_chars = false;
    bool multiline = false;
    for (char c : string) {
        if (c < ' ' || c == '"' || c == '\'') {
            has_spec_chars = true;
            if (multiline) {
                break;
            }
        }
        if (c == '\n') {
            multiline = true;
            break;
        }
    }
    if (multiline) {
        ss << "|-\n";
        size_t offset = 0;
        size_t newoffset = 0;

        do {
            offset = newoffset;
            if (offset == string.length() - 1 && string[offset] == '\n') {
                break;
            }
            add_indent(ss, indent);
            newoffset = string.find('\n', offset + 1);
            if (newoffset == std::string::npos) {
                ss << string.substr(offset);
                break;
            } else {
                ss << string.substr(offset + 1, newoffset - offset - 1);
            }
            ss << '\n';
        } while (true);
    } else {
        if (has_spec_chars || string.empty()) {
            ss << util::escape(string, false);
        } else {
            ss << string;
        }
    }
}

static void to_string(
    std::stringstream& ss,
    const dv::value& value,
    int indent,
    bool eliminateIndent = false
) {
    using dv::value_type;

    switch (value.getType()) {
        case value_type::string:
            insert_string(ss, value.asString(), indent);
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
        case value_type::object: {
            if (value.empty()) {
                ss << "{}";
                break;
            }
            bool first = true;
            for (const auto& [key, elem] : value.asObject()) {
                if (!first) {
                    ss << '\n';   
                }
                if (!eliminateIndent) {
                    add_indent(ss, indent);
                } else {
                    eliminateIndent = false;
                }
                ss << key << ": ";
                if ((elem.isObject() || elem.isList()) && !elem.empty()) {
                    ss << "\n";
                    to_string(ss, elem, indent + 1);
                } else {
                    to_string(ss, elem, indent + 1);
                }
                first = false;
            }
            break;
        }
        case value_type::list: {
            if (value.empty()) {
                ss << "[]";
                break;
            }
            bool first = true;
            for (const auto& elem : value) {
                if (!first) {
                    ss << '\n';   
                }
                if (!eliminateIndent) {
                    add_indent(ss, indent);
                } else {
                    eliminateIndent = false;
                }
                ss << "- ";
                to_string(ss, elem, indent + 1, true);
                first = false;
            }
            break;
        }
        case value_type::bytes: {
            const auto& bytes = value.asBytes();
            auto b64 = util::base64_encode(bytes.data(), bytes.size());
            b64 = util::join(util::split_by_n(b64, 64), '\n');
            insert_string(ss, b64, indent);
            break;
        }
    }
}

std::string yaml::stringify(const dv::value& value) {
    std::stringstream ss;
    to_string(ss, value, 0);
    return ss.str();
}
