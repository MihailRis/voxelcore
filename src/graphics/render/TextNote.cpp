#include "TextNote.hpp"

TextNote::TextNote(std::wstring text, NotePreset preset, glm::vec3 position)
    : text(std::move(text)),
      preset(std::move(preset)),
      position(std::move(position)) {
}

void TextNote::setText(std::wstring_view text) {
    this->text = text;
}

void TextNote::setEntity(entityid_t eid) {
    this->entity = eid;
}

entityid_t TextNote::getEntity() const {
    return entity;
}

const std::wstring& TextNote::getText() const {
    return text;
}

const NotePreset& TextNote::getPreset() const {
    return preset;
}

void TextNote::updatePreset(const dv::value& data) {
    preset.deserialize(data);
}

void TextNote::setPosition(const glm::vec3& position) {
    this->position = position;
}

const glm::vec3& TextNote::getPosition() const {
    return position;
}

const glm::vec3& TextNote::getAxisX() const {
    return xAxis;
}

const glm::vec3& TextNote::getAxisY() const {
    return yAxis;
}

void TextNote::setAxisX(const glm::vec3& vec) {
    xAxis = vec;
}

void TextNote::setAxisY(const glm::vec3& vec) {
    yAxis = vec;
}
