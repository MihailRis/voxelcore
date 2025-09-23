#include "util/stringutil.hpp"
#include "coders/BasicParser.hpp"

#include <gtest/gtest.h>

TEST(stringutil, crop_utf8) {
    // Project source files must be UTF-8 encoded
    std::string str = u8"пример";
    str = str.substr(0, util::crop_utf8(str, 7));
    EXPECT_EQ(str, u8"при");
}

TEST(stringutil, utf8) {
    std::string str = u8"テキストデモ";
    auto u32str = util::str2u32str_utf8(str);
    std::string str2 = util::u32str2str_utf8(u32str);
    EXPECT_EQ(str, str2);
}

static std::wstring gen_random_unicode_wstring(int n) {
    std::wstring str;
    str.resize(n);
    for (int i = 0; i < n; i++) {
        // wstring is 16 bit in some systems
        str[i] = rand() & 0xFFFF;
    }
    return str;
}

TEST(stringutil, utf8_random) {
    srand(5436324);

    auto str = gen_random_unicode_wstring(10'000);
    auto utf8str = util::wstr2str_utf8(str);
    auto back = util::str2wstr_utf8(utf8str);
    EXPECT_EQ(str, back);
}

TEST(stringutil, base64) {
    srand(2019);
    for (size_t size = 0; size < 30; size++) {
        auto bytes = std::make_unique<ubyte[]>(size);
        for (int i = 0; i < size; i++) {
            bytes[i] = rand();
        }
        auto base64 = util::base64_encode(bytes.get(), size);
        auto decoded = util::base64_decode(base64);
        ASSERT_EQ(size, decoded.size());
        for (size_t i = 0; i < size; i++) {
            ASSERT_EQ(bytes[i], decoded[i]);
        }
    }
}

TEST(stringutil, base64_urlsafe) {
    srand(2019);
    for (size_t size = 0; size < 30; size++) {
        auto bytes = std::make_unique<ubyte[]>(size);
        for (int i = 0; i < size; i++) {
            bytes[i] = rand();
        }
        auto base64 = util::base64_urlsafe_encode(bytes.get(), size);
        auto decoded = util::base64_urlsafe_decode(base64);
        ASSERT_EQ(size, decoded.size());
        for (size_t i = 0; i < size; i++) {
            ASSERT_EQ(bytes[i], decoded[i]);
        }
    }
}

class StringParser : BasicParser<char> {
public:
    StringParser(std::string_view source) : BasicParser("<string>", source) {}

    std::string parse() {
        ++pos;
        return parseString(source[0], true);
    }
};

TEST(stringutil, escape_cases) {
    auto escaped = util::escape("тест5", true);
    auto expected = "\"\\u0442\\u0435\\u0441\\u04425\"";
    ASSERT_EQ(expected, escaped);

    srand(345873458);
    for (int i = 0; i < 36; i++) {
        rand();
    }

    auto str = gen_random_unicode_wstring(40);
    auto utf8str = util::wstr2str_utf8(str);
    escaped = util::escape(utf8str, true);

    StringParser parser(escaped);
    auto restored = parser.parse();
    for (int i = 0; i < utf8str.length(); i++) {
        if (utf8str[i] != restored[i]) {
            std::cout << i << ": " << (int)utf8str[i] << " " << (int)restored[i] << std::endl;
        }
    }
    EXPECT_EQ(utf8str, restored);
}
