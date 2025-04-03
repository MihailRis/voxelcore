#include "data/StructLayout.hpp"

#include <gtest/gtest.h>
#include <algorithm>
#include <climits>

#include "data/dv.hpp"

using namespace data;

TEST(StructLayout, ReadWrite) {
    ubyte buffer[16] {};

    std::vector<Field> fields {
        Field {FieldType::I8, "a", 1},
        Field {FieldType::CHAR, "s", 4},
        Field {FieldType::F32, "f", 1},
    };
    auto layout = StructLayout::create(fields);
    EXPECT_EQ(layout.size(), 9);

    layout.setInteger(buffer, 42, "a");
    EXPECT_EQ(layout.getInteger(buffer, "a"), 42);

    layout.setNumber(buffer, 3.141592f, "f");
    EXPECT_FLOAT_EQ(layout.getNumber(buffer, "f"), 3.141592f);

    layout.setAscii(buffer, "hello", "s");
    EXPECT_EQ(layout.getChars(buffer, "s"), "hell");
}

TEST(StructLayout, Unicode) {
    ubyte buffer[8] {};
    std::vector<Field> fields {
        Field {FieldType::CHAR, "text", 5},
    };
     auto layout = StructLayout::create(fields);
     EXPECT_EQ(layout.size(), 5);

     layout.setUnicode(buffer, u8"テキストデモ", "text");
     EXPECT_EQ(layout.getChars(buffer, "text"), std::string(u8"テ"));

     layout.setUnicode(buffer, u8"пример", "text");
     EXPECT_EQ(layout.getChars(buffer, "text"), std::string(u8"пр"));
}

TEST(StructLayout, ConvertReorder) {
    ubyte src[16] {};
    std::vector<Field> srcFields {
        Field {FieldType::CHAR, "text", 5},
        Field {FieldType::F64, "pi", 1},
    };
    auto srcLayout = StructLayout::create(srcFields);
    srcLayout.setAscii(src, "truth", "text");
    srcLayout.setNumber(src, 3.141592, "pi");

    EXPECT_EQ(srcLayout.getChars(src, "text"), "truth");
    EXPECT_DOUBLE_EQ(srcLayout.getNumber(src, "pi"), 3.141592);

    ubyte dst[32] {};
    std::vector<Field> dstFields {
        Field {FieldType::I64, "arr", 2}, // an array of 2 i64
        Field {FieldType::CHAR, "text", 5},
        Field {FieldType::F64, "pi", 1},
    };
    auto dstLayout = StructLayout::create(dstFields);
    EXPECT_EQ(srcLayout.checkCompatibility(dstLayout).size(), 0);
    dstLayout.convert(srcLayout, src, dst, false);

    EXPECT_EQ(dstLayout.getChars(dst, "text"), "truth");
    EXPECT_DOUBLE_EQ(dstLayout.getNumber(dst, "pi"), 3.141592);
}

TEST(StructLayout, ConvertWithLoss) {
    ubyte src[32] {};
    std::vector<Field> srcFields {
        Field {FieldType::CHAR, "text", 5},
        Field {FieldType::I16, "someint", 1},
        Field {FieldType::F64, "pi", 1},
    };
    auto srcLayout = StructLayout::create(srcFields);
    srcLayout.setAscii(src, "truth", "text");
    srcLayout.setInteger(src, 150, "someint");
    srcLayout.setNumber(src, 3.141592, "pi");

    EXPECT_EQ(srcLayout.getChars(src, "text"), "truth");
    EXPECT_EQ(srcLayout.getInteger(src, "someint"), 150);
    EXPECT_DOUBLE_EQ(srcLayout.getNumber(src, "pi"), 3.141592);

    ubyte dst[8] {};
    std::vector<Field> dstFields {
        Field {FieldType::CHAR, "text", 3},
        Field {FieldType::I8, "someint", 1, FieldConvertStrategy::CLAMP},
    };
    auto dstLayout = StructLayout::create(dstFields);
    auto report = srcLayout.checkCompatibility(dstLayout);

    // check report
    std::sort(report.begin(), report.end(), [](const auto& a, const auto& b) {
        return a.name < b.name;
    });
    EXPECT_EQ(report.size(), 3);

    EXPECT_EQ(report[0].name, "pi");
    EXPECT_EQ(report[0].type, FieldIncapatibilityType::MISSING);

    EXPECT_EQ(report[1].name, "someint");
    EXPECT_EQ(report[1].type, FieldIncapatibilityType::DATA_LOSS);

    EXPECT_EQ(report[2].name, "text");
    EXPECT_EQ(report[2].type, FieldIncapatibilityType::DATA_LOSS);

    // convert with losses
    dstLayout.convert(srcLayout, src, dst, true);

    EXPECT_EQ(dstLayout.getChars(dst, "text"), "tru");
    EXPECT_EQ(dstLayout.getInteger(dst, "someint"), INT8_MAX);
}

TEST(StructLayout, Serialization) {
    std::vector<Field> fields {
        Field (FieldType::CHAR, "text", 5),
        Field (FieldType::I16, "someint", 1),
        Field (FieldType::F64, "pi", 1),
    };
    auto layout1 = StructLayout::create(fields);
    auto serialized = layout1.serialize();

    std::cout << serialized << std::endl;

    StructLayout layout2;
    layout2.deserialize(serialized);

    EXPECT_EQ(layout1, layout2);
}
