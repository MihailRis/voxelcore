#pragma once

#include "typedefs.hpp"
#include "util/EnumMetadata.hpp"

class Content;
class ContentPackRuntime;

enum class ContentType { NONE, BLOCK, ITEM, ENTITY, GENERATOR };

VC_ENUM_METADATA(ContentType)
    {"none", ContentType::NONE},
    {"block", ContentType::BLOCK},
    {"item", ContentType::ITEM},
    {"entity", ContentType::ENTITY},
    {"generator", ContentType::GENERATOR},
VC_ENUM_END

enum class ResourceType : size_t {
    CAMERA,
    POST_EFFECT_SLOT,
    LAST = POST_EFFECT_SLOT
};

inline constexpr auto RESOURCE_TYPES_COUNT =
    static_cast<size_t>(ResourceType::LAST) + 1;

VC_ENUM_METADATA(ResourceType)
    {"camera", ResourceType::CAMERA},
    {"post-effect-slot", ResourceType::POST_EFFECT_SLOT},
VC_ENUM_END
