#include "syntax_parser.hpp"

#include <set>

#include "data/dv.hpp"
#include "util/stringutil.hpp"
#include "BasicParser.hpp"

using namespace devtools;

dv::value Syntax::serialize() const {
    auto map = dv::object();
    map["language"] = language;
    map["line-comment"] = util::wstr2str_utf8(lineComment);
    map["multiline-comment-start"] = util::wstr2str_utf8(multilineCommentStart);
    map["multiline-comment-end"] = util::wstr2str_utf8(multilineCommentEnd);
    map["multiline-string-start"] = util::wstr2str_utf8(multilineStringStart);
    map["multiline-string-end"] = util::wstr2str_utf8(multilineStringEnd);

    auto& extsList = map.list("extensions");
    for (const auto& ext : extensions) {
        extsList.add(ext);
    }

    auto& keywordsList = map.list("keywords");
    for (const auto& keyword : keywords) {
        keywordsList.add(util::wstr2str_utf8(keyword));
    }
    return map;
}

void Syntax::deserialize(const dv::value& src) {
    src.at("language").get(language);

    std::string lineComment;
    std::string multilineCommentStart;
    std::string multilineCommentEnd;
    std::string multilineStringStart;
    std::string multilineStringEnd;
    src.at("line-comment").get(lineComment);
    src.at("multiline-comment-start").get(multilineCommentStart);
    src.at("multiline-comment-end").get(multilineCommentEnd);
    src.at("multiline-string-start").get(multilineStringStart);
    src.at("multiline-string-end").get(multilineStringEnd);
    this->lineComment = util::str2wstr_utf8(lineComment);
    this->multilineCommentStart = util::str2wstr_utf8(multilineCommentStart);
    this->multilineCommentEnd = util::str2wstr_utf8(multilineCommentEnd);
    this->multilineStringStart = util::str2wstr_utf8(multilineStringStart);
    this->multilineStringEnd = util::str2wstr_utf8(multilineStringEnd);

    if (src.has("extensions")) {
        const auto& extsList = src["extensions"];
        for (const auto& ext : extsList) {
            extensions.insert(ext.asString());
        }
    }

    if (src.has("keywords")) {
        const auto& keywordsList = src["keywords"];
        for (const auto& keyword : keywordsList) {
            keywords.insert(util::str2wstr_utf8(keyword.asString()));
        }
    }
}

inline bool is_common_identifier_start(int c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '_';
}

inline bool is_common_identifier_part(int c) {
    return is_common_identifier_start(c) || is_digit(c);
}

inline bool is_lua_operator_start(int c) {
    return c == '=' || c == '~' || c == '+' || c == '-' || c == '/' || c == '*'
        || c == '%' || c == '^' || c == '#' || c == '<' || c == '>' || c == ':'
        || c == '.';
}

class Tokenizer : BasicParser<wchar_t> {
    const Syntax& syntax;
    std::vector<Token> tokens;
public:
    Tokenizer(
        const Syntax& syntax, std::string_view file, std::wstring_view source
    )
        : BasicParser(file, source), syntax(syntax) {
    }

    std::wstring parseLuaName() {
        char c = peek();
        if (!is_identifier_start(c)) {
            throw error("identifier expected");
        }
        int start = pos;
        while (hasNext() && is_identifier_part(source[pos])) {
            pos++;
        }
        return std::wstring(source.substr(start, pos - start));
    }

    inline Location currentLocation() const {
        return Location {
            static_cast<int>(pos),
            static_cast<int>(linestart),
            static_cast<int>(line)};
    }

    void emitToken(
        TokenTag tag, std::wstring name, Location start, bool standalone=false
    ) {
        tokens.emplace_back(
            tag,
            std::move(name),
            std::move(start),
            currentLocation()
        );
        if (standalone) skip(1);
    }

    /// @brief Get next operator token without checking operator for existing
    std::wstring parseOperator() {
        int start = pos;
        wchar_t first = peek();
        switch (first) {
            case '#': case '+': case '/': case '*': case '^':
            case '%':
                skip(1);
                return std::wstring({first});
            case '-':
                skip(1);
                if (hasNext() && peekNoJump() == '-') {
                    skip(1);
                    return L"--";
                }
                return std::wstring({first});
        }
        skip(1);
        char second = peekNoJump();
        if ((first == '=' && second == '=') || (first == '~' && second == '=') ||
            (first == '<' && second == '=') || (first == '>' && second == '=')) {
            skip(1);
            return std::wstring(source.substr(start, pos - start));
        }
        if (first == '.' && second == '.') {
            skip(1);
            if (peekNoJump() == '.') {
                skip(1);
            }
        }
        return std::wstring(source.substr(start, pos - start));
    }

    void emitLineComment(devtools::Location start) {
        auto string = readUntilEOL();
        emitToken(TokenTag::COMMENT, std::wstring(string), start);
        skipLine();
    }

    std::vector<Token> tokenize() {
        skipWhitespace();
        while (hasNext()) {
            skipWhitespace();
            if (!hasNext()) {
                continue;
            }
            wchar_t c = peek();
            auto start = currentLocation();
            if (is_common_identifier_start(c)) {
                auto name = parseLuaName();
                TokenTag tag =
                    (syntax.keywords.find(name) == syntax.keywords.end()
                         ? TokenTag::NAME
                         : TokenTag::KEYWORD);
                emitToken(
                    tag,
                    std::move(name),
                    start
                );
                continue;
            } else if (is_digit(c)) {
                dv::value value;
                auto tag = TokenTag::UNEXPECTED;
                try {
                    value = parseNumber(1);
                    tag = value.isInteger() ? TokenTag::INTEGER
                                            : TokenTag::NUMBER;
                } catch (const parsing_error& err) {}

                auto literal = source.substr(start.pos, pos - start.pos);
                emitToken(tag, std::wstring(literal), start);
                continue;
            }
            const auto& mcommentStart = syntax.multilineCommentStart;
            if (!mcommentStart.empty() && c == mcommentStart[0] &&
                isNext(syntax.multilineCommentStart)) {
                auto string = readUntil(syntax.multilineCommentEnd, true);
                skip(syntax.multilineCommentEnd.length());
                emitToken(
                    TokenTag::COMMENT,
                    std::wstring(string) + syntax.multilineCommentEnd,
                    start
                );
                continue;
            }
            const auto& mstringStart = syntax.multilineStringStart;
            if (!mstringStart.empty() && c == mstringStart[0] &&
                isNext(syntax.multilineStringStart)) {
                skip(mstringStart.length());
                auto string = readUntil(syntax.multilineStringEnd, true);
                skip(syntax.multilineStringEnd.length());
                emitToken(TokenTag::STRING, std::wstring(string), start);
                continue;
            }
            switch (c) {
                case '(': case '[': case '{': 
                    emitToken(TokenTag::OPEN_BRACKET, std::wstring({c}), start, true);
                    continue;
                case ')': case ']': case '}': 
                    emitToken(TokenTag::CLOSE_BRACKET, std::wstring({c}), start, true);
                    continue;
                case ',':
                    emitToken(TokenTag::COMMA, std::wstring({c}), start, true);
                    continue;
                case ';':
                    emitToken(TokenTag::SEMICOLON, std::wstring({c}), start, true);
                    continue;
                case '\'': case '"': {
                    skip(1);
                    auto string = parseString(c, false);
                    emitToken(TokenTag::STRING, std::move(string), start);
                    continue;
                }
                default: break;
            }
            if (is_lua_operator_start(c)) {
                auto text = parseOperator();
                if (text == syntax.lineComment) {
                    emitLineComment(start);
                    continue;
                }
                emitToken(TokenTag::OPERATOR, std::move(text), start);
                continue;
            }
            auto text = readUntilWhitespace();
            if (text.find(syntax.lineComment) == 0) {
                emitLineComment(start);
                continue;
            }
            emitToken(TokenTag::UNEXPECTED, std::wstring(text), start);
        }
        return std::move(tokens);
    }
};

std::vector<Token> devtools::tokenize(
    const Syntax& syntax, std::string_view file, std::wstring_view source
) {
    return Tokenizer(syntax, file, source).tokenize();
}
