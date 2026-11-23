#include <gtest/gtest.h>

#include "physics/Hitbox.hpp"

TEST(Hitbox, AABBRelativeToOrigin) {
    for (double coord : {0.0, 1e6, 3e7}) {
        worldvec3 position(coord + 0.3, 64.25, -coord - 0.7);
        Hitbox hitbox(
            1, BodyType::DYNAMIC, position, glm::vec3(0.3f, 0.9f, 0.3f)
        );
        AABB aabb = hitbox.getAABB(glm::floor(position));
        EXPECT_NEAR(aabb.a.x, 0.0f, 1e-6f) << "at " << coord;
        EXPECT_NEAR(aabb.a.y, -0.65f, 1e-6f) << "at " << coord;
        EXPECT_NEAR(aabb.a.z, 0.0f, 1e-6f) << "at " << coord;
        EXPECT_NEAR(aabb.b.x, 0.6f, 1e-6f) << "at " << coord;
        EXPECT_NEAR(aabb.b.y, 1.15f, 1e-6f) << "at " << coord;
        EXPECT_NEAR(aabb.b.z, 0.6f, 1e-6f) << "at " << coord;
    }
}

TEST(Hitbox, SetPosFarFromOrigin) {
    worldvec3 position(3e7 + 0.3, 64.25, -3e7 - 0.7);
    Hitbox hitbox(1, BodyType::DYNAMIC, {}, glm::vec3(0.3f, 0.9f, 0.3f));
    hitbox.setPos(position);
    EXPECT_EQ(hitbox.position, position);
    EXPECT_EQ(hitbox.prevPosition, position);

    hitbox.setPos(position + worldvec3(0.05, 0.0, 0.0));
    EXPECT_EQ(hitbox.prevPosition, position);
}
