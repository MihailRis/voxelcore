#pragma once

#include <string>
#include <vector>

#include "typedefs.hpp"
#include "util/Buffer.hpp"

namespace util {
    /// @brief Function used for string serialization in text formats
    std::string escape(std::string_view s, bool escapeUnicode=true);

    /// @brief Function used for error messages
    std::string quote(const std::string& s);

    std::wstring lfill(std::wstring s, uint length, wchar_t c);
    std::wstring rfill(std::wstring s, uint length, wchar_t c);

    uint encode_utf8(uint32_t c, ubyte* bytes);
    uint32_t decode_utf8(uint& size, const char* bytes);

    /// @brief Encode raw wstring to UTF-8
    /// @param ws source raw wstring
    /// @return new UTF-8 encoded string
    std::string wstr2str_utf8(std::wstring_view ws);

    /// @brief Decode UTF-8 string
    /// @param s source encoded string
    /// @return new raw decoded wstring
    std::wstring str2wstr_utf8(std::string_view s);

    /// @brief Encode raw u32string to UTF-8
    /// @param ws source raw wstring
    /// @return new UTF-8 encoded string
    std::string u32str2str_utf8(std::u32string_view ws);

    /// @brief Decode UTF-8 string
    /// @param s source encoded string
    /// @return new raw decoded u32string
    std::u32string str2u32str_utf8(const std::string& s);

    inline std::string str2str_utf8(std::string_view s) {
        return std::string(s);
    }
    inline std::string str2str_utf8(std::wstring_view s) {
        return wstr2str_utf8(s);
    }
    inline std::string str2str_utf8(std::u32string_view s) {
        return u32str2str_utf8(s);
    }

    /// @brief Calculated length of UTF-8 encoded string that fits into maxSize
    /// @param s source UTF-8 encoded string view
    /// @param maxSize max encoded string length after crop
    /// @return cropped string size (less or equal to maxSize)
    size_t crop_utf8(std::string_view s, size_t maxSize);

    /// @brief Measure utf8-encoded string length
    /// @param s source encoded string
    /// @return unicode string length (number of codepoints)
    size_t length_utf8(std::string_view s);

    size_t length_utf8(std::wstring_view s);
    
    bool is_integer(const std::string& text);
    bool is_integer(const std::wstring& text);
    bool is_valid_filename(const std::wstring& name);

    void ltrim(std::string& s);
    void rtrim(std::string& s);
    void trim(std::string& s);

    std::string to_string(double x);
    std::wstring to_wstring(double x, int precision);

    std::string base64_encode(const ubyte* data, size_t size);
    std::string base64_urlsafe_encode(const ubyte* data, size_t size);

    util::Buffer<ubyte> base64_decode(const char* str, size_t size);
    util::Buffer<ubyte> base64_urlsafe_decode(const char* str, size_t size);
    util::Buffer<ubyte> base64_decode(std::string_view str);
    util::Buffer<ubyte> base64_urlsafe_decode(std::string_view str);

    std::string tohex(uint64_t value);

    std::string mangleid(uint64_t value);

    int replaceAll(
        std::string& str, const std::string& from, const std::string& to
    );

    int replaceAll(
        std::wstring& str, const std::wstring& from, const std::wstring& to
    );

    double parse_double(const std::string& str);
    double parse_double(const std::string& str, size_t offset, size_t len);

    std::string capitalized(const std::string& str);

    std::wstring lower_case(const std::wstring& str);
    std::wstring upper_case(const std::wstring& str);
    std::wstring capitalized(const std::wstring& str);
    std::wstring pascal_case(const std::wstring& str);

    /// @brief Convert `any_prefix:some_data_id` to `some data id`. Leaves
    /// '_' characters at end of the id.
    /// @param id source id
    /// @return resulting caption or empty string if there's nothing but prefix
    std::string id_to_caption(const std::string& id);

    std::vector<std::string> split(const std::string& str, char delimiter);
    std::vector<std::wstring> split(const std::wstring& str, char delimiter);

    std::string format_data_size(size_t size);

    std::pair<std::string, std::string> split_at(std::string_view view, char c);

    template <typename CharT>
    std::vector<std::basic_string<CharT>> split_by_n(
        const std::basic_string<CharT>& str, size_t n
    ) {
        std::vector<std::basic_string<CharT>> result;
        for (size_t i = 0; i < str.length(); i += n) {
            result.push_back(str.substr(i, n));
        }
        return result;
    }

    template <typename CharT>
    std::basic_string<CharT> join(
        const std::vector<std::basic_string<CharT>>& strings, CharT delimiter
    ) {
        std::basic_stringstream<CharT> ss;
        for (size_t i = 0; i < strings.size(); ++i) {
            ss << strings[i];
            if (i != strings.size() - 1) {
                ss << delimiter;
            }
        }
        return ss.str();
    }
}
