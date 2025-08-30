#include "BasicParser.hpp"

#include <cmath>
#include <sstream>

#include "util/stringutil.hpp"

namespace {
    inline int is_box(int c) {
        switch (c) {
            case 'B':
            case 'b':
                return 2;
            case 'O':
            case 'o':
                return 8;
            case 'X':
            case 'x':
                return 16;
        }
        return 10;
    }

    inline double power(double base, int64_t power) {
        double result = 1.0;
        for (int64_t i = 0; i < power; i++) {
            result *= base;
        }
        return result;
    }
}

template<typename CharT>
void BasicParser<CharT>::skipWhitespaceBasic(bool newline) {
    while (hasNext()) {
        CharT next = source[pos];
        if (next == '\n') {
            if (!newline) {
                break;
            }
            line++;
            linestart = ++pos;
            continue;
        }
        if (is_whitespace(next)) {
            pos++;
        } else {
            break;
        }
    }
}

template<typename CharT>
void BasicParser<CharT>::skipWhitespace(bool newline) {
    if (hashComment) {
        skipWhitespaceHashComment(newline);
        return;
    } else if (clikeComment) {
        skipWhitespaceCLikeComment(newline);
        return;
    }
    skipWhitespaceBasic(newline);
}

template<typename CharT>
void BasicParser<CharT>::skipWhitespaceHashComment(bool newline) {
    skipWhitespaceBasic(newline);
    if (hasNext() && source[pos] == '#') {
        if (!newline) {
            readUntilEOL();
            return;
        }
        skipLine();
        if (hasNext() && (is_whitespace(source[pos]) || source[pos] == '#')) {
            skipWhitespaceHashComment(newline);
        }
    }
}

template<typename CharT>
void BasicParser<CharT>::skipWhitespaceCLikeComment(bool newline) {
    skipWhitespaceBasic(newline);
    if (hasNext() && source[pos] == '/' && pos + 1 < source.length()) {
        pos++;
        switch (source[pos]) {
            case '*':
                pos++;
                while (hasNext()) {
                    if (source[pos] == '/' && source[pos-1] == '*') {
                        pos++;
                        skipWhitespace();
                        return;
                    }
                    pos++;
                }
                break;
            case '/':
                if (!newline) {
                    readUntilEOL();
                    return;
                }
                skipLine();
                if (hasNext() && (is_whitespace(source[pos]) || source[pos] == '/')) {
                    skipWhitespaceCLikeComment(newline);
                }
                break;
            default:
                pos--;
                break;
        }
    }
}

template<typename CharT>
void BasicParser<CharT>::skip(size_t n) {
    n = std::min(n, source.length() - pos);

    for (size_t i = 0; i < n; i++) {
        char next = source[pos++];
        if (next == '\n') {
            line++;
            linestart = pos;
        }
    }
}

template<typename CharT>
void BasicParser<CharT>::skipLine() {
    while (hasNext()) {
        if (source[pos] == '\n') {
            pos++;
            linestart = pos;
            line++;
            break;
        }
        pos++;
    }
}

template<typename CharT>
void BasicParser<CharT>::skipEmptyLines() {
    skipWhitespace();
    pos = linestart;
}

template<typename CharT>
bool BasicParser<CharT>::skipTo(const std::basic_string<CharT>& substring) {
    size_t idx = source.find(substring, pos);
    if (idx == std::string::npos) {
        skip(source.length() - pos);
        return false;
    } else {
        skip(idx - pos);
        return true;
    }
}

template<typename CharT>
bool BasicParser<CharT>::hasNext() {
    return pos < source.length();
}

template<typename CharT>
size_t BasicParser<CharT>::remain() const {
    return source.length() - pos;
}

template<typename CharT>
bool BasicParser<CharT>::isNext(const std::basic_string<CharT>& substring) {
    if (substring.empty()) {
        return false;
    }
    if (source.length() - pos < substring.length()) {
        return false;
    }
    return source.substr(pos, substring.length()) == substring;
}

template<typename CharT>
CharT BasicParser<CharT>::nextChar() {
    if (!hasNext()) {
        throw error("unexpected end");
    }
    return source[pos++];
}

template<typename CharT>
void BasicParser<CharT>::expect(CharT expected) {
    char c = peek();
    if (c != expected) {
        throw error(
            "'" + std::string({static_cast<char>(expected)}) + "' expected"
        );
    }
    pos++;
}

template<typename CharT>
void BasicParser<CharT>::expect(const std::basic_string<CharT>& substring) {
    if (substring.empty()) return;
    for (uint i = 0; i < substring.length(); i++) {
        if (source.length() <= pos + i || source[pos + i] != substring[i]) {
            throw error(
                util::quote(util::str2str_utf8(substring)) + " expected"
            );
        }
    }
    pos += substring.length();
}

template<typename CharT>
void BasicParser<CharT>::expectNewLine() {
    while (hasNext()) {
        char next = source[pos];
        if (next == '\n') {
            line++;
            linestart = ++pos;
            return;
        }
        if (is_whitespace(next)) {
            pos++;
        } else {
            throw error("line separator expected");
        }
    }
}

template<typename CharT>
void BasicParser<CharT>::goBack(size_t count) {
    if (pos < count) {
        throw std::runtime_error("pos < jump");
    }
    if (pos) {
        pos -= count;
    }
}

template<typename CharT> void BasicParser<CharT>::reset() {
    pos = 0;
}

template<typename CharT>
CharT BasicParser<CharT>::peekInLine() {
    while (hasNext()) {
        CharT next = source[pos];
        if (next == '\n') {
            return next;
        }
        if (is_whitespace(next)) {
            pos++;
        } else {
            break;
        }
    }
    if (pos >= source.length()) {
        throw error("unexpected end");
    }
    return source[pos];
}

template<typename CharT>
CharT BasicParser<CharT>::peek() {
    skipWhitespace();
    if (pos >= source.length()) {
        throw error("unexpected end");
    }
    return source[pos];
}

template<typename CharT>
CharT BasicParser<CharT>::peekNoJump() {
    if (pos >= source.length()) {
        throw error("unexpected end");
    }
    return source[pos];
}

template<typename CharT>
std::basic_string_view<CharT> BasicParser<CharT>::readUntil(CharT c) {
    int start = pos;
    while (hasNext() && source[pos] != c) {
        pos++;
    }
    return source.substr(start, pos - start);
}

template <typename CharT>
std::basic_string_view<CharT> BasicParser<CharT>::readUntil(
    std::basic_string_view<CharT> s, bool nothrow
) {
    int start = pos;
    size_t found = source.find(s, pos);
    if (found == std::string::npos) {
        if (nothrow) {
            pos = source.size();
            return source.substr(start);
        }
        throw error(util::quote(util::str2str_utf8(s)) + " expected");
    }
    skip(found - pos);
    return source.substr(start, pos - start);
}

template <typename CharT>
std::basic_string_view<CharT> BasicParser<CharT>::readUntilWhitespace() {
    int start = pos;
    while (hasNext() && !is_whitespace(source[pos])) {
        pos++;
    }
    return source.substr(start, pos - start);
}

template <typename CharT>
std::basic_string_view<CharT> BasicParser<CharT>::readUntilEOL() {
    int start = pos;
    while (hasNext() && source[pos] != '\n') {
        pos++;
    }
    if (pos > start && source[pos - 1] == '\r') {
        return source.substr(start, pos - start - 1);
    }
    return source.substr(start, pos - start);
}

template <typename CharT>
std::basic_string<CharT> BasicParser<CharT>::parseName() {
    char c = peek();
    if (!is_identifier_start(c)) {
        throw error("identifier expected");
    }
    int start = pos;
    while (hasNext() && is_identifier_part(source[pos])) {
        pos++;
    }
    return std::basic_string<CharT>(source.substr(start, pos - start));
}

template <typename CharT>
std::basic_string<CharT> BasicParser<CharT>::parseXmlName() {
    CharT c = peek();
    if (!is_json_identifier_start(c)) {
        throw error("identifier expected");
    }
    int start = pos;
    while (hasNext() && is_json_identifier_part(source[pos])) {
        pos++;
    }
    return std::basic_string<CharT>(source.substr(start, pos - start));
}

template <typename CharT>
int64_t BasicParser<CharT>::parseSimpleInt(int base) {
    CharT c = peek();
    int index = hexchar2int(c);
    if (index == -1 || index >= base) {
        throw error("invalid number literal");
    }
    int64_t value = index;
    pos++;
    while (hasNext()) {
        c = source[pos];
        while (c == '_') {
            c = source[++pos];
        }
        index = hexchar2int(c);
        if (index == -1 || index >= base) {
            return value;
        }
        value *= base;
        value += index;
        pos++;
    }
    return value;
}

template <typename CharT>
dv::value BasicParser<CharT>::parseNumber() {
    switch (peek()) {
        case '-':
            skip(1);
            return parseNumber(-1);
        case '+':
            skip(1);
            return parseNumber(1);
        default:
            return parseNumber(1);
    }
}

template <typename CharT>
dv::value BasicParser<CharT>::parseNumber(int sign) {
    CharT c = peek();
    int base = 10;
    if (c == '0' && pos + 1 < source.length() &&
        (base = is_box(source[pos + 1])) != 10) {
        pos += 2;
        return parseSimpleInt(base);
    } else if (c == 'i' && pos + 2 < source.length() && source[pos + 1] == 'n' && source[pos + 2] == 'f') {
        pos += 3;
        return INFINITY * sign;
    } else if (c == 'n' && pos + 2 < source.length() && source[pos + 1] == 'a' && source[pos + 2] == 'n') {
        pos += 3;
        return NAN * sign;
    }
    int64_t value = parseSimpleInt(base);
    if (!hasNext()) {
        return value * sign;
    }
    c = source[pos];
    if (c == 'e' || c == 'E') {
        pos++;
        int s = 1;
        if (peek() == '-') {
            s = -1;
            pos++;
        } else if (peek() == '+') {
            pos++;
        }
        return sign * value * power(10.0, s * parseSimpleInt(10));
    }
    if (c == '.') {
        pos++;
        int64_t expo = 1;
        while (hasNext() && source[pos] == '0') {
            expo *= 10;
            pos++;
        }
        int64_t afterdot = 0;
        if (hasNext() && is_digit(source[pos])) {
            afterdot = parseSimpleInt(10);
        }
        expo *= power(
            10, std::max(
                static_cast<int64_t>(0),
                static_cast<int64_t>(std::log10(afterdot) + 1)
            )
        );

        double dvalue = (value + (afterdot / (double)expo));
        if (hasNext()){
            c = source[pos];
            if (c == 'e' || c == 'E') {
                pos++;
                int s = 1;
                if (peek() == '-') {
                    s = -1;
                    pos++;
                } else if (peek() == '+') {
                    pos++;
                }
                return sign * dvalue * power(10.0, s * parseSimpleInt(10));
            }
        }
        return sign * dvalue;
    }
    return sign * value;
}

template <typename CharT>
std::basic_string<CharT> BasicParser<CharT>::parseString(
    CharT quote, bool closeRequired
) {
    std::basic_stringstream<CharT> ss;
    while (hasNext()) {
        CharT c = source[pos];
        if (c == quote) {
            pos++;
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
            if (c == 'u' || c == 'x') {
                int codepoint = parseSimpleInt(16);
                ubyte bytes[4];
                int size = util::encode_utf8(codepoint, bytes);
                CharT chars[4];
                for (int i = 0; i < 4; i++) {
                    chars[i] = bytes[i];
                }
                ss.write(chars, size);
                continue;
            }
            switch (c) {
                case 'n': ss << '\n'; break;
                case 'r': ss << '\r'; break;
                case 'b': ss << '\b'; break;
                case 't': ss << '\t'; break;
                case 'f': ss << '\f'; break;
                case 'v': ss << '\v'; break;
                case '\'': ss << '\''; break;
                case '"': ss << '"'; break;
                case '\\': ss << '\\'; break;
                case '/': ss << '/'; break;
                case '\n': continue;
                default:
                    throw error(
                        "'\\" +
                        util::str2str_utf8(std::basic_string<CharT>({c})) +
                        "' is an illegal escape"
                    );
            }
            continue;
        }
        if (c == '\n' && closeRequired) {
            throw error("non-closed string literal");
        }
        ss << c;
        pos++;
    }
    if (closeRequired) {
        throw error("unexpected end");
    }
    return ss.str();
}

template <>
inline parsing_error BasicParser<char>::error(const std::string& message) {
    return parsing_error(message, filename, source, pos, line, linestart);
}

template <>
inline parsing_error BasicParser<wchar_t>::error(const std::string& message) {
    size_t utf8pos = util::length_utf8(source.substr(0, pos));
    size_t utf8linestart =
        utf8pos - util::length_utf8(source.substr(linestart, pos));
    return parsing_error(
        message,
        filename,
        util::str2str_utf8(source),
        utf8pos,
        line,
        utf8linestart
    );
}
