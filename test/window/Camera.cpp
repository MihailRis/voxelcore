#include <gtest/gtest.h>

#include <cmath>
#include <glm/ext.hpp>

#include "window/Camera.hpp"

TEST(Camera, ViewRotationFarFromOrigin) {
    for (float coord : {0.0f, 1e5f, 1e6f, 3e7f}) {
        for (bool rotated : {false, true}) {
            Camera camera(glm::vec3(coord, 64.0f, coord), 1.0f);
            if (rotated) {
                camera.rotate(glm::radians(-20.0f), glm::radians(35.0f), 0.0f);
            }
            glm::mat4 rotation = camera.getView(false);
            glm::mat4 view = camera.getView();
            for (int col = 0; col < 4; col++) {
                for (int row = 0; row < 4; row++) {
                    ASSERT_TRUE(std::isfinite(view[col][row]))
                        << "at " << coord;
                }
            }
            for (int col = 0; col < 3; col++) {
                for (int row = 0; row < 3; row++) {
                    EXPECT_NEAR(rotation[col][row], view[col][row], 1e-5f)
                        << "at " << coord << ", rotated " << rotated;
                }
            }
        }
    }
}
