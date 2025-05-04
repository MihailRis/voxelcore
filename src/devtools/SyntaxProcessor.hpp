#pragma once

#include <set>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>

struct FontStylesScheme;

namespace devtools {
    struct Syntax;

    enum SyntaxStyles {
        DEFAULT, KEYWORD, LITERAL, COMMENT, ERROR
    };

    class SyntaxProcessor {
    public:
        std::unique_ptr<FontStylesScheme> highlight(
            const std::string& ext, std::wstring_view source
        ) const;

        void addSyntax(std::unique_ptr<Syntax> syntax);
    private:
        std::vector<std::unique_ptr<Syntax>> langs;
        std::unordered_map<std::string, const Syntax*> langsExtensions;
    };
}
