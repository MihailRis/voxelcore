#include <gtest/gtest.h>

#include "coders/xml.hpp"
#include "coders/commons.hpp"

TEST(XML, VCM) {
    std::string code = ""
        "@line x1 y1 z1\n"
        "# @tst ysfdrg\n"
        "@box {\n"
        "  @rect texture \"$2\"\n"
        "  @utro a 53.1\n"
        "}\n"
    ;
    std::cout << code << std::endl;

    try {
        auto document = xml::parse_vcm("<test>", code, "test");
        std::cout << xml::stringify(*document);
    } catch (const parsing_error& err) {
        std::cerr << err.errorLog() << std::endl;
        throw err;
    }
}
