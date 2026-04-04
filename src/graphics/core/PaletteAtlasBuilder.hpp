#pragma once

#include <memory>
#include <map>
#include <string>
#include <glm/vec4.hpp>

class Atlas;

class PaletteAtlasBuilder {
public:
    struct Entry {
        glm::vec4 color;
    };

    PaletteAtlasBuilder();

    void put(const std::string& name, const glm::vec4& color);

    std::unique_ptr<Atlas> build() const;
private:
    std::map<std::string, Entry> entries;
};
