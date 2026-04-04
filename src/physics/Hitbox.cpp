#include "Hitbox.hpp"

Hitbox::Hitbox(
    entityid_t entity, BodyType type, glm::vec3 position, glm::vec3 halfsize
)
    : entity(entity),
      type(type),
      position(position),
      halfsize(halfsize),
      velocity(0.0f, 0.0f, 0.0f),
      prevPosition(position) {
}
