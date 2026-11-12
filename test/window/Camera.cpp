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

TEST(Camera, ViewRelativeToOrigin) {
    for (double coord : {0.0, 500.3, -1e5 - 0.3, 1e6 + 0.3, 3e7 + 0.3}) {
        Camera camera(worldvec3(coord, 64.0, -coord), 1.0f);
        camera.rotate(glm::radians(-20.0f), glm::radians(35.0f), 0.0f);

        worldvec3 origin = camera.getOrigin();
        worldvec3 local = camera.getLocalPosition();
        EXPECT_LE(glm::abs(local.x), 512.0) << "at " << coord;
        EXPECT_LE(glm::abs(local.z), 512.0) << "at " << coord;
        EXPECT_NEAR(camera.position.x, origin.x + local.x, 1e-4) << coord;
        EXPECT_NEAR(camera.position.z, origin.z + local.z, 1e-4) << coord;

        worldvec3 point = camera.position + worldvec3(camera.front) * 10.0;
        glm::vec4 viewPoint =
            camera.getView() * glm::vec4(glm::vec3(point - origin), 1.0f);
        EXPECT_NEAR(viewPoint.x, 0.0f, 1e-3f) << "at " << coord;
        EXPECT_NEAR(viewPoint.y, 0.0f, 1e-3f) << "at " << coord;
        EXPECT_NEAR(viewPoint.z, -10.0f, 1e-3f) << "at " << coord;
    }
}
