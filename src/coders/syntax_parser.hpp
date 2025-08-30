#pragma once

#include <set>
#include <string>
#include <vector>

#include "devtools/syntax.hpp"
#include "interfaces/Serializable.hpp"

namespace devtools {
    struct Syntax : Serializable {
        std::string language;
        std::set<std::string> extensions;
        std::set<std::wstring> keywords;
        std::wstring lineComment;
        std::wstring multilineCommentStart;
        std::wstring multilineCommentEnd;
        std::wstring multilineStringStart;
        std::wstring multilineStringEnd;
    
        dv::value serialize() const override;
        void deserialize(const dv::value& src) override;
    };

    std::vector<Token> tokenize(
        const Syntax& syntax, std::string_view file, std::wstring_view source
    );
}
