#include "Hitbox.hpp"

#include <stdexcept>

Hitbox::Hitbox(BodyType type, glm::vec3 position, glm::vec3 halfsize) 
  : type(type),
    position(position), 
    halfsize(halfsize), 
    velocity(0.0f,0.0f,0.0f)
{}
