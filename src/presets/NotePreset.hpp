#pragma once

#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#include <optional>
#include <string>

#include "interfaces/Serializable.hpp"

enum class NoteDisplayMode {
    STATIC_BILLBOARD,
    Y_FREE_BILLBOARD,
    XY_FREE_BILLBOARD,
    PROJECTED
};

std::string to_string(NoteDisplayMode mode);
std::optional<NoteDisplayMode> NoteDisplayMode_from(std::string_view s);

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
