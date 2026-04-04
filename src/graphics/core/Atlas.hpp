#pragma once

#include <set>
#include <string>
#include <memory>
#include <vector>
#include <optional>
#include <unordered_map>

#include "maths/UVRegion.hpp"
#include "typedefs.hpp"

class ImageData;
class Texture;

class Atlas {
    std::shared_ptr<Texture> texture;
    std::shared_ptr<ImageData> image;
    std::unordered_map<std::string, UVRegion> regions;
public:
    /// @param image atlas raster
    /// @param regions atlas regions
    /// @param prepare generate texture (.prepare())
    Atlas(
        std::unique_ptr<ImageData> image, 
        std::unordered_map<std::string, UVRegion> regions, 
        bool prepare
    );
    ~Atlas();

    void prepare();

    bool has(const std::string& name) const;
    const UVRegion& get(const std::string& name) const;
    std::optional<UVRegion> getIf(const std::string& name) const;

    Texture* getTexture() const;
    ImageData* getImage() const;

    std::shared_ptr<Texture> shareTexture() const;
    std::shared_ptr<ImageData> shareImageData() const;
};

struct atlasentry {
    std::string name;
    std::shared_ptr<ImageData> image;
};

class AtlasBuilder {
    std::vector<atlasentry> entries;
    std::set<std::string> names;
public:
    AtlasBuilder() = default;
    void add(const std::string& name, std::unique_ptr<ImageData> image);
    bool has(const std::string& name) const;
    const std::set<std::string>& getNames() { return names; };

    /// @brief Build atlas from all added images
    /// @param extrusion textures extrusion pixels 
    /// (greater is less mip-mapping artifacts)
    /// @param prepare generate atlas texture (calls .prepare()) 
    /// @param maxResolution max atlas resolution
    std::unique_ptr<Atlas> build(uint extrusion, bool prepare=true, uint maxResolution=0);
};
