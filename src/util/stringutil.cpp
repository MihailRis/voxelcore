#include "stringutil.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>

std::string util::escape(std::string_view s, bool escapeUnicode) {
    std::stringstream ss;
    ss << '"';
    size_t pos = 0;
    while (pos < s.length()) {
        char c = s[pos];
        switch (c) {
            case '\n':
                ss << "\\n";
                break;
            case '\r':
                ss << "\\r";
                break;
            case '\t':
                ss << "\\t";
                break;
            case '\f':
                ss << "\\f";
                break;
            case '\b':
                ss << "\\b";
                break;
            case '"':
                ss << "\\\"";
                break;
            case '\\':
                ss << "\\\\";
                break;
            default:
                if (c & 0x80) {
                    uint cpsize;
                    int codepoint = decode_utf8(cpsize, s.data() + pos);
                    if (escapeUnicode) {
                        ss << "\\u" << std::setw(4) << std::setfill('0') << std::hex << codepoint;
                    } else {
                        ss << std::string(s.data() + pos, cpsize);
                    }
                    pos += cpsize-1;
                    break;
                }
                if (c < ' ') {
                    ss << "\\" << std::oct << uint(ubyte(c));
                    break;
                }
                ss << c;
                break;
        }
        pos++;
    }
    ss << '"';
    return ss.str();
}

std::string util::quote(const std::string& s) {
    return escape(s, false);
}

std::wstring util::lfill(std::wstring s, uint length, wchar_t c) {
    if (s.length() >= length) {
        return s;
    }
    std::wstringstream ss;
    for (uint i = 0; i < length - s.length(); i++) {
        ss << c;
    }
    ss << s;
    return ss.str();
}

std::wstring util::rfill(std::wstring s, uint length, wchar_t c) {
    if (s.length() >= length) {
        return s;
    }
    std::wstringstream ss;
    ss << s;
    for (uint i = 0; i < length - s.length(); i++) {
        ss << c;
    }
    return ss.str();
}

static size_t length_utf8_codepoint(uint32_t c) {
    if (c < 0x80) {
        return 1;
    } else if (c < 0x0800) {
        return 2;
    } else if (c < 0x010000) {
        return 3;
    } else {
        return 4;
    }
}

uint util::encode_utf8(uint32_t c, ubyte* bytes) {
    if (c < 0x80) {
        bytes[0] = ((c >> 0) & 0x7F) | 0x00;
        return 1;
    } else if (c < 0x0800) {
        bytes[0] = ((c >> 6) & 0x1F) | 0xC0;
        bytes[1] = ((c >> 0) & 0x3F) | 0x80;
        return 2;
    } else if (c < 0x010000) {
        bytes[0] = ((c >> 12) & 0x0F) | 0xE0;
        bytes[1] = ((c >> 6) & 0x3F) | 0x80;
        bytes[2] = ((c >> 0) & 0x3F) | 0x80;
        return 3;
    } else {
        bytes[0] = ((c >> 18) & 0x07) | 0xF0;
        bytes[1] = ((c >> 12) & 0x3F) | 0x80;
        bytes[2] = ((c >> 6) & 0x3F) | 0x80;
        bytes[3] = ((c >> 0) & 0x3F) | 0x80;
        return 4;
    }
}

struct utf_t {
    char mask;
    char lead;
    uint32_t beg;
    uint32_t end;
    int bits_stored;
};

const utf_t utf[] = {
    /* mask             lead              beg      end     bits */
    {(char)0b00111111, (char)0b10000000, 0, 0, 6},
    {(char)0b01111111, (char)0b00000000, 0000, 0177, 7},
    {(char)0b00011111, (char)0b11000000, 0200, 03777, 5},
    {(char)0b00001111, (char)0b11100000, 04000, 0177777, 4},
    {(char)0b00000111, (char)0b11110000, 0200000, 04177777, 3},
    {0, 0, 0, 0, 0},
};

inline uint utf8_len(ubyte cp) {
    if ((cp & 0x80) == 0) {
        return 1;
    }
    if ((cp & 0xE0) == 0xC0) {
        return 2;
    }
    if ((cp & 0xF0) == 0xE0) {
        return 3;
    }
    if ((cp & 0xF8) == 0xF0) {
        return 4;
    }
    throw std::runtime_error("utf8 decode error");
}

uint32_t util::decode_utf8(uint& size, const char* chr) {
    size = utf8_len(*chr);
    int shift = utf[0].bits_stored * (size - 1);
    uint32_t code = (*chr++ & utf[size].mask) << shift;

    for (uint i = 1; i < size; ++i, ++chr) {
        shift -= utf[0].bits_stored;
        code |= ((char)*chr & utf[0].mask) << shift;
    }
    return code;
}

size_t util::crop_utf8(std::string_view s, size_t maxSize) {
    size_t pos = 0;
    uint size = 0;
    while (pos < s.length()) {
        decode_utf8(size, s.data() + pos);
        if (pos + size > maxSize) {
            return pos;
        }
        pos += size;
    }
    return pos;
}

size_t util::length_utf8(std::string_view s) {
    size_t length = 0;
    size_t pos = 0;
    while (pos < s.length()) {
        pos += utf8_len(s[pos]);
        length++;
    }
    return length;
}

size_t util::length_utf8(std::wstring_view s) {
    size_t length = 0;
    for (size_t i = 0; i < s.length(); i++) {
        length += length_utf8_codepoint(s[i]);
    }
    return length;
}

template<class C>
std::string xstr2str_utf8(std::basic_string_view<C> xs) {
    std::vector<char> chars;
    ubyte buffer[4];
    for (C xc : xs) {
        uint size = util::encode_utf8(
            static_cast<uint>(xc), buffer);
        for (uint i = 0; i < size; i++) {
            chars.push_back(buffer[i]);
        }
    }
    return std::string(chars.data(), chars.size());
}

std::string util::wstr2str_utf8(std::wstring_view ws) {
    return xstr2str_utf8(ws);
}

std::string util::u32str2str_utf8(std::u32string_view ws) {
    return xstr2str_utf8(ws);
}

template<class C>
std::basic_string<C> str2xstr_utf8(std::string_view s) {
    std::vector<C> chars;
    size_t pos = 0;
    uint size = 0;
    while (pos < s.length()) {
        chars.push_back(util::decode_utf8(size, &s.at(pos)));
        pos += size;
    }
    return std::basic_string<C>(chars.data(), chars.size());
}

std::wstring util::str2wstr_utf8(std::string_view s) {
    return str2xstr_utf8<wchar_t>(s);
}

std::u32string util::str2u32str_utf8(const std::string& s) {
    return str2xstr_utf8<char32_t>(s);
}

bool util::is_integer(const std::string& text) {
    for (char c : text) {
        if (c < '0' || c > '9') return false;
    }
    return true;
}

bool util::is_integer(const std::wstring& text) {
    for (wchar_t c : text) {
        if (c < L'0' || c > L'9') return false;
    }
    return true;
}

bool util::is_valid_filename(const std::wstring& name) {
    for (wchar_t c : name) {
        if (c < 31 || c == '/' || c == '\\' || c == '<' || c == '>' ||
            c == ':' || c == '"' || c == '|' || c == '?' || c == '*') {
            return false;
        }
    }
    return true;
}

void util::ltrim(std::string& s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
                return !std::isspace(ch);
            }));
}

void util::rtrim(std::string& s) {
    s.erase(
        std::find_if(
            s.rbegin(),
            s.rend(),
            [](unsigned char ch) { return !std::isspace(ch); }
        ).base(),
        s.end()
    );
}

void util::trim(std::string& s) {
    rtrim(s);
    ltrim(s);
}

std::string util::to_string(double x) {
    std::stringstream ss;
    ss << std::setprecision(6);
    ss << x;
    return ss.str();
}

std::wstring util::to_wstring(double x, int precision) {
    std::wstringstream ss;
    ss << std::fixed << std::setprecision(precision) << x;
    return ss.str();
}

const char B64ABC[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "+/";

const char URLSAFE_B64ABC[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "-_";

inline ubyte base64_decode_char(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return 0;
}

inline ubyte base64_urlsafe_decode_char(char c) {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '-') return 62;
    if (c == '_') return 63;
    return 0;
}

template <const char* ABC>
static void base64_encode_(const ubyte* segment, char* output) {
    output[0] = ABC[(segment[0] & 0b11111100) >> 2];
    output[1] =
        ABC[((segment[0] & 0b11) << 4) | ((segment[1] & 0b11110000) >> 4)];
    output[2] =
        ABC[((segment[1] & 0b1111) << 2) | ((segment[2] & 0b11000000) >> 6)];
    output[3] = ABC[segment[2] & 0b111111];
}

template <const char* ABC>
static std::string base64_encode_impl(const ubyte* data, size_t size) {
    std::stringstream ss;

    size_t fullsegments = (size / 3) * 3;

    size_t i = 0;
    for (; i < fullsegments; i += 3) {
        char output[] = "====";
        base64_encode_<ABC>(data + i, output);
        ss << output;
    }

    ubyte ending[3] {};
    for (; i < size; i++) {
        ending[i - fullsegments] = data[i];
    }
    size_t trailing = size - fullsegments;
    if (trailing) {
        char output[] = "====";
        output[0] = ABC[(ending[0] & 0b11111100) >> 2];
        output[1] =
            ABC[((ending[0] & 0b11) << 4) | ((ending[1] & 0b11110000) >> 4)];
        if (trailing > 1)
            output[2] =
                ABC[((ending[1] & 0b1111) << 2) |
                    ((ending[2] & 0b11000000) >> 6)];
        if (trailing > 2) output[3] = ABC[ending[2] & 0b111111];
        ss << output;
    }
    return ss.str();
}

std::string util::base64_encode(const ubyte* data, size_t size) {
    return base64_encode_impl<B64ABC>(data, size);
}

std::string util::base64_urlsafe_encode(const ubyte* data, size_t size) {
    return base64_encode_impl<URLSAFE_B64ABC>(data, size);
}

std::string util::tohex(uint64_t value) {
    std::stringstream ss;
    ss << std::hex << value;
    return ss.str();
}

std::string util::mangleid(uint64_t value) {
    // todo: use base64
    return tohex(value);
}

template <ubyte(base64_decode_char)(char)>
static util::Buffer<ubyte> base64_decode_impl(const char* str, size_t size) {
    util::Buffer<ubyte> bytes((size / 4) * 3);
    ubyte* dst = bytes.data();
    for (size_t i = 0; i < (size / 4) * 4;) {
        ubyte a = base64_decode_char(ubyte(str[i++]));
        ubyte b = base64_decode_char(ubyte(str[i++]));
        ubyte c = base64_decode_char(ubyte(str[i++]));
        ubyte d = base64_decode_char(ubyte(str[i++]));
        *(dst++) = ((a << 2) | ((b & 0b110000) >> 4));
        *(dst++) = (((b & 0b1111) << 4) | ((c & 0b111100) >> 2));
        *(dst++) = (((c & 0b11) << 6) | d);
    }
    if (size >= 2) {
        size_t outsize = bytes.size();
        if (str[size - 1] == '=') outsize--;
        if (str[size - 2] == '=') outsize--;
        bytes.resizeFast(outsize);
    }
    return bytes;
}

util::Buffer<ubyte> util::base64_urlsafe_decode(const char* str, size_t size) {
    return base64_decode_impl<base64_urlsafe_decode_char>(str, size);
}

util::Buffer<ubyte> util::base64_decode(const char* str, size_t size) {
    return base64_decode_impl<base64_decode_char>(str, size);
}

util::Buffer<ubyte> util::base64_urlsafe_decode(std::string_view str) {
    return base64_urlsafe_decode(str.data(), str.size());
}

util::Buffer<ubyte> util::base64_decode(std::string_view str) {
    return base64_decode(str.data(), str.size());
}

template <typename CharT>
static int replace_all(
    std::basic_string<CharT>& str, 
    const std::basic_string<CharT>& from, 
    const std::basic_string<CharT>& to
) {
    int count = 0;
    size_t offset = 0;
    while (true) {
        size_t start_pos = str.find(from, offset);
        if (start_pos == std::basic_string<CharT>::npos) {
            break;
        }
        str.replace(start_pos, from.length(), to);
        offset = start_pos + to.length();
        count++;
    }
    return count;
}

int util::replaceAll(
    std::string& str, const std::string& from, const std::string& to
) {
    return replace_all(str, from, to);
}

int util::replaceAll(
    std::wstring& str, const std::wstring& from, const std::wstring& to
) {
    return replace_all(str, from, to);
}

// replace it with std::from_chars in the far far future
double util::parse_double(const std::string& str) {
    std::istringstream ss(str);
    ss.imbue(std::locale("C"));
    double d;
    ss >> d;
    if (ss.fail()) {
        throw std::runtime_error("invalid number format");
    }
    return d;
}

double util::parse_double(const std::string& str, size_t offset, size_t len) {
    return parse_double(str.substr(offset, len));
}

std::wstring util::lower_case(const std::wstring& str) {
    std::wstring result = str;
    static const std::locale loc("");
    for (uint i = 0; i < result.length(); i++) {
        result[i] = static_cast<wchar_t>(std::tolower(str[i], loc));
    }
    return result;
}

std::wstring util::upper_case(const std::wstring& str) {
    std::wstring result = str;
    static const std::locale loc("");
    for (uint i = 0; i < result.length(); i++) {
        result[i] = static_cast<wchar_t>(std::toupper(str[i], loc));
    }
    return result;
}

std::wstring util::capitalized(const std::wstring& str) {
    if (str.empty()) return str;
    static const std::locale loc("");
    return std::wstring({static_cast<wchar_t>(std::toupper(str[0], loc))}) +
           str.substr(1);
}

std::string util::capitalized(const std::string& str) {
    if (str.empty()) return str;
    static const std::locale loc("");
    return std::string({static_cast<char>(std::toupper(str[0], loc))}) +
           str.substr(1);
}

std::wstring util::pascal_case(const std::wstring& str) {
    if (str.empty()) return str;
    static const std::locale loc("");
    std::wstring result = str;
    bool upper = true;
    for (uint i = 0; i < result.length(); i++) {
        auto c = result[i];
        if (c <= ' ') {
            upper = true;
        } else if (upper) {
            result[i] = static_cast<wchar_t>(std::toupper(str[i], loc));
            upper = false;
        }
    }
    return result;
}

std::string util::id_to_caption(const std::string& id) {
    std::string result = id;

    size_t index = result.find(':');
    if (index < result.length() - 1) {
        result = result.substr(index + 1);
    } else {
        return "";
    }
    size_t offset = 0;
    for (; offset < result.length() && result[offset] == '_'; offset++) {
    }

    for (; offset < result.length(); offset++) {
        if (result[offset] == '_') {
            result[offset] = ' ';
        }
    }
    return result;
}

/// @brief Split string by delimiter
/// @param str source string
/// @param delimiter split delimiter size
/// @return vector of string parts, containing at least one string
std::vector<std::string> util::split(const std::string& str, char delimiter) {
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string tmp;
    while (std::getline(ss, tmp, delimiter)) {
        result.push_back(tmp);
    }
    if (result.empty()) {
        result.emplace_back("");
    }
    return result;
}

/// @brief Split wstring by delimiter
/// @param str source string
/// @param delimiter split delimiter size
/// @return vector of string parts, containing at least one string
std::vector<std::wstring> util::split(const std::wstring& str, char delimiter) {
    std::vector<std::wstring> result;
    std::wstringstream ss(str);
    std::wstring tmp;
    while (std::getline(ss, tmp, static_cast<wchar_t>(delimiter))) {
        result.push_back(tmp);
    }
    if (result.empty()) {
        result.emplace_back(L"");
    }
    return result;
}

std::string util::format_data_size(size_t size) {
    if (size < 1024) {
        return std::to_string(size) + " B";
    }
    const std::string postfixes[] {
        " B", " KiB", " MiB", " GiB", " TiB", " EiB", " PiB"};
    int group = 0;
    size_t remainder = 0;
    while (size >= 1024) {
        group++;
        remainder = size % 1024;
        size /= 1024;
    }
    return std::to_string(size) + "." +
           std::to_string(static_cast<int>(round(remainder / 1024.0f))) +
           postfixes[group];
}

std::pair<std::string, std::string> util::split_at(
    std::string_view view, char c
) {
    size_t idx = view.find(c);
    if (idx == std::string::npos) {
        throw std::runtime_error(util::quote(std::string({c})) + " not found");
    }
    return std::make_pair(
        std::string(view.substr(0, idx)), std::string(view.substr(idx + 1))
    );
}
