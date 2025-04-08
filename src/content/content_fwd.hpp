#pragma once

#include "typedefs.hpp"

class Content;
class ContentPackRuntime;

enum class ContentType { NONE, BLOCK, ITEM, ENTITY, GENERATOR };

enum class ResourceType : size_t {
    CAMERA,
    POST_EFFECT_SLOT,
    LAST = POST_EFFECT_SLOT
};

inline constexpr auto RESOURCE_TYPES_COUNT =
    static_cast<size_t>(ResourceType::LAST) + 1;
