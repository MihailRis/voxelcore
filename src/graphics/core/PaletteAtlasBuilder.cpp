#include "PaletteAtlasBuilder.hpp"

#include "../core/Atlas.hpp"
#include "../core/ImageData.hpp"

PaletteAtlasBuilder::PaletteAtlasBuilder() = default;

void PaletteAtlasBuilder::put(const std::string& name, const glm::vec4& color) {
    entries[name] = { glm::clamp(color, glm::vec4(0.0f), glm::vec4(1.0f)) };
}

std::unique_ptr<Atlas> PaletteAtlasBuilder::build() const {
    AtlasBuilder builder;
    for (const auto& [name, entry] : entries) {
        ubyte data[4] {
            static_cast<ubyte>(entry.color.r * 255),
            static_cast<ubyte>(entry.color.g * 255),
            static_cast<ubyte>(entry.color.b * 255),
            static_cast<ubyte>(entry.color.a * 255),
        };
        builder.add(name, std::make_unique<ImageData>(
            ImageFormat::RGBA8888, 1, 1, data
        ));
    }
    return builder.build(1, true);
}
