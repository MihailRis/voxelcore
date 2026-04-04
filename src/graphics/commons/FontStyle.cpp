#include "FontStyle.hpp"

#include "data/dv.hpp"
#include "data/dv_util.hpp"
#include "devtools/SyntaxProcessor.hpp"

FontStyle FontStyle::parse(const dv::value& src) {
    FontStyle style {};
    src.at("bold").get(style.bold);
    src.at("italic").get(style.italic);
    src.at("strikethrough").get(style.strikethrough);
    src.at("underline").get(style.underline);
    dv::get_vec(src, "color", style.color);
    return style;
}

static void parse_style(
    const dv::value& src,
    FontStylesScheme& scheme,
    const std::string& name,
    devtools::SyntaxStyles tag
) {
    if (src.has(name)) {
        scheme.palette[static_cast<int>(tag)] = FontStyle::parse(src[name]);
    }
}

FontStylesScheme FontStylesScheme::parse(const dv::value& src) {
    FontStylesScheme scheme {};
    scheme.palette.resize(8);
    parse_style(src, scheme, "default", devtools::SyntaxStyles::DEFAULT);
    parse_style(src, scheme, "keyword", devtools::SyntaxStyles::KEYWORD);
    parse_style(src, scheme, "literal", devtools::SyntaxStyles::LITERAL);
    parse_style(src, scheme, "comment", devtools::SyntaxStyles::COMMENT);
    parse_style(src, scheme, "error", devtools::SyntaxStyles::ERROR);
    return scheme;
}
