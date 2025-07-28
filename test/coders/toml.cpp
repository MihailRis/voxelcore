#include "coders/toml.hpp"

#include <gtest/gtest.h>

#include "data/dv.hpp"
#include "util/stringutil.hpp"
#include "util/Buffer.hpp"
#include "coders/commons.hpp"

TEST(TOML, EncodeDecode) {
    const std::string name = "TOML-encoder";
    const int bytesSize = 20;
    const int year = 2019;
    const float score = 3.141592;
    const bool visible = true;
    dv::objects::Bytes srcBytes(bytesSize);
    for (int i = 0; i < bytesSize; i ++) {
        srcBytes[i] = rand();
    }

    std::string text;
    {
        auto object = dv::object();
        object["name"] = name;
        object["year"] = year;
        object["score"] = score;
        object["visible"] = visible;
        object["data"] = srcBytes;
        object["values"] = dv::list({
            5, 3, std::string("hello"), dv::object({{"number", 1234}})
        });

        text = toml::stringify(object, "");
        std::cout << text << std::endl;
    }
    try {
        auto object = toml::parse("[string]", text);
        EXPECT_EQ(object["name"].asString(), name);
        EXPECT_EQ(object["year"].asInteger(), year);
        EXPECT_FLOAT_EQ(object["score"].asNumber(), score);
        EXPECT_EQ(object["visible"].asBoolean(), visible);
        auto b64string = object["data"].asString();

        auto bytes = util::base64_decode(b64string);
        EXPECT_EQ(bytes.size(), bytesSize);
        for (int i = 0; i < bytesSize; i++) {
            EXPECT_EQ(bytes[i], srcBytes[i]);
        }
    } catch (const parsing_error& err) {
        std::cerr << err.errorLog() << std::endl;
        throw;
    }
}

// Modified example from toml.io
inline std::string SRC_EXAMPLE =
    "# This is a TOML document\n"
    "\n"
    "title = \"TOML Example\"\n"
    "\n"
    "[owner]\n"
    "name = \"Tom Preston-Werner\"\n"
    "dob = 1979-05-27T07:32:00-08:00\n"
    "\n"
    "[database]\n"
    "enabled = true\n"
    "ports = [ 8000, 8001, 8002 ]\n"
    "data = [ [\"delta\", \"phi\"], [3.14] ]\n"
    "temp_targets = { cpu = 79.5, case = 72.0 }\n"
    "\n"
    "[servers]\n"
    "\n"
    "[servers.alpha]\n"
    "ip = \"10.0.0.1\"\n"
    "role = \"frontend\"\n"
    "\n"
    "[servers.beta]\n"
    "ip = \"10.0.0.2\"\n"
    "role = \"\"\"back\\\n"
    "end\"\"\"\n"
    "\n"
    "[[users]]\n"
    "name = \"noname\"\n"
    "\n"
    "[[users]]\n"
    "name = \"user1\"\n"
    "suspended = true\n";

TEST(TOML, ExampleCode) {
    try {
        std::cout << SRC_EXAMPLE << std::endl;
        auto object = toml::parse("[string]", SRC_EXAMPLE);
        std::cout << object << std::endl;
    } catch (const parsing_error& err) {
        std::cerr << err.errorLog() << std::endl;
        throw;
    }
}
