#pragma once

#include "constants.hpp"
#include "presets/NotePreset.hpp"

/// @brief 3D text instance
class TextNote {
    std::wstring text;
    NotePreset preset;
    glm::vec3 position;
    entityid_t entity = ENTITY_NONE;

    glm::vec3 xAxis {1, 0, 0};
    glm::vec3 yAxis {0, 1, 0};
public:
    TextNote(std::wstring text, NotePreset preset, glm::vec3 position);

    void setText(std::wstring_view text);
    void setEntity(entityid_t eid);
    entityid_t getEntity() const;

    const std::wstring& getText() const;

    const NotePreset& getPreset() const;

    void updatePreset(const dv::value& data);

    void setPosition(const glm::vec3& position);
    const glm::vec3& getPosition() const;

    const glm::vec3& getAxisX() const;
    const glm::vec3& getAxisY() const;

    void setAxisX(const glm::vec3& vec);
    void setAxisY(const glm::vec3& vec);
};
