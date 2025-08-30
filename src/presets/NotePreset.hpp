#pragma once

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <string>

#include "interfaces/Serializable.hpp"
#include "util/EnumMetadata.hpp"

enum class NoteDisplayMode {
    STATIC_BILLBOARD,
    Y_FREE_BILLBOARD,
    XY_FREE_BILLBOARD,
    PROJECTED
};

VC_ENUM_METADATA(NoteDisplayMode)
    {"static_billboard", NoteDisplayMode::STATIC_BILLBOARD},
    {"y_free_billboard", NoteDisplayMode::Y_FREE_BILLBOARD},
    {"xy_free_billboard", NoteDisplayMode::XY_FREE_BILLBOARD},
    {"projected", NoteDisplayMode::PROJECTED},
VC_ENUM_END

struct NotePreset : public Serializable {
    NoteDisplayMode displayMode = NoteDisplayMode::STATIC_BILLBOARD;
    glm::vec4 color {1.0f};
    float scale = 1.0f;
    float renderDistance = 32.0f;
    float xrayOpacity = 0.0f;
    float perspective = 1.0f;

    dv::value serialize() const override;
    void deserialize(const dv::value& src) override;
};
