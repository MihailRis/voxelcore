#include <gtest/gtest.h>
#include <atomic>

#include "world/generator/SurroundMap.hpp"

TEST(SurroundMap, InitTest) {
    int maxLevelZone = 50;
    int x = 0;
    int y = 0;
    int8_t maxLevel = 5;

    SurroundMap map(maxLevelZone, maxLevel);
    std::atomic_int affected = 0;

    map.setLevelCallback(1, [&affected](auto, auto) {
        affected++;
    });
    map.setCenter(0, 0);
    map.completeAt(x, y);
    EXPECT_EQ(affected, (maxLevel * 2 - 1) * (maxLevel * 2 - 1));

    for (int ly = -maxLevel + 1; ly < maxLevel; ly++) {
        for (int lx = -maxLevel + 1; lx < maxLevel; lx++) {
            int levelExpected = maxLevel - std::max(std::abs(lx), std::abs(ly));
            EXPECT_EQ(map.at(x + lx, y + ly), levelExpected);
        }
    }

    affected = 0;
    map.completeAt(x - 1, y);
    EXPECT_EQ(affected, maxLevel * 2 - 1);
}

#define VISUAL_TEST
#ifdef VISUAL_TEST

#include <glm/gtc/random.hpp>

#include "coders/png.hpp"
#include "graphics/core/ImageData.hpp"

void visualize(const SurroundMap& map, int mul, int max) {
    const auto& areaMap = map.getArea();
    int w = areaMap.getWidth();
    int h = areaMap.getHeight();
    int ox = areaMap.getOffsetX();
    int oy = areaMap.getOffsetY();
    
    ImageData image(ImageFormat::RGB888, w, h);
    ubyte* bytes = image.getData();
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            int val = areaMap.get(x + ox, y + oy) * mul;
            if (val && val / mul < max) {
                val = val / 4 + 50;
            }
            bytes[(y * w + x) * 3] = val;
            bytes[(y * w + x) * 3 + 1] = val;
            bytes[(y * w + x) * 3 + 2] = val;
        }
    }
    png::write_image("test.png", &image);
}

TEST(SurroundMap, Visualize) {
    int levels = 3;
    SurroundMap map(50, levels);
    map.setCenter(0, 0);
    
    for (int i = 0; i < 1000; i++) {
        float x = glm::gaussRand(0.0f, 2.0f);
        float y = glm::gaussRand(0.0f, 2.0f);
        map.completeAt(x, y);
    }
    visualize(map, 30, levels);
}

#endif
