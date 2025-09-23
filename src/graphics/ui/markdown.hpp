#pragma once

#include <string>
#include <memory>
#include <sstream>

struct FontStylesScheme;

// VoxelCore Markdown dialect

namespace markdown {

    template <typename CharT>
    struct Result {
        /// @brief Text with erased markdown
        std::basic_string<CharT> text;
        /// @brief Text styles scheme
        std::unique_ptr<FontStylesScheme> styles;
    };

    Result<char> process(std::string_view source, bool eraseMarkdown);
    Result<wchar_t> process(std::wstring_view source, bool eraseMarkdown);

    template <typename CharT>
    inline std::basic_string<CharT> escape(std::basic_string_view<CharT> source) {
        std::basic_stringstream<CharT> ss;
        int pos = 0;
        while (pos < source.size()) {
            CharT first = source[pos];
            if (first == '\\' && pos + 1 < source.size()) {
                CharT second = source[++pos];
                ss << first << second;
                pos++;
                continue;
            } else if (first == '*' || first == '~' || first == '_') {
                ss << '\\';
            }
            ss << first;
            pos++;
        }
        return ss.str();
    }
}
