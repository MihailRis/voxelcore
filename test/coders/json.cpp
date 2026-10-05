#include <gtest/gtest.h>

#include "coders/json.hpp"
#include "util/stringutil.hpp"

TEST(JSON, EncodeDecode) {
    const std::string name = "JSON-encoder";
    const int bytesSize = 20;
    const int year = 2019;
    const float score = 3.141592;
    const double distance = 1.23e45;
    const double epsilon = 1e-6;
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
        object["distance"] = distance;
        object["epsilon"] = epsilon;
        object["visible"] = visible;
        object["data"] = srcBytes;

        text = json::stringify(object, false, "");
    }
    {
        auto object = json::parse(text);
        EXPECT_EQ(object["name"].asString(), name);
        EXPECT_EQ(object["year"].asInteger(), year);
        EXPECT_FLOAT_EQ(object["score"].asNumber(), score);
        EXPECT_DOUBLE_EQ(object["distance"].asNumber(), distance);
        EXPECT_DOUBLE_EQ(object["epsilon"].asNumber(), epsilon);
        EXPECT_EQ(object["visible"].asBoolean(), visible);
        auto b64string = object["data"].asString();

        auto bytes = util::base64_decode(b64string);
        EXPECT_EQ(bytes.size(), bytesSize);
        for (int i = 0; i < bytesSize; i++) {
            EXPECT_EQ(bytes[i], srcBytes[i]);
        }
    }
}
