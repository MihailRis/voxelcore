#include "Atlas.hpp"

#include "Texture.hpp"
#include "ImageData.hpp"
#include "maths/LMPacker.hpp"

#include <stdexcept>

Atlas::Atlas(
    std::unique_ptr<ImageData> image, 
    std::unordered_map<std::string, UVRegion> regions,
    bool prepare
) : texture(nullptr),
    image(std::move(image)),
    regions(regions) 
{        
    if (prepare) {
        this->prepare();
    }
}

Atlas::~Atlas() = default;

void Atlas::prepare() {
    texture = Texture::from(image.get());
}

bool Atlas::has(const std::string& name) const {
    return regions.find(name) != regions.end();
}

const UVRegion& Atlas::get(const std::string& name) const {
    return regions.at(name);
}

std::optional<UVRegion> Atlas::getIf(const std::string& name) const {
    const auto& found = regions.find(name);
    if (found == regions.end()) {
        return std::nullopt;
    }
    return found->second;
}

Texture* Atlas::getTexture() const {
    return texture.get();
}

ImageData* Atlas::getImage() const {
    return image.get();
}

std::shared_ptr<Texture> Atlas::shareTexture() const {
    return texture;
}

std::shared_ptr<ImageData> Atlas::shareImageData() const {
    return image;
}

void AtlasBuilder::add(const std::string& name, std::unique_ptr<ImageData> image) {
    entries.push_back(atlasentry{name, std::shared_ptr<ImageData>(image.release())});
    names.insert(name);
}

bool AtlasBuilder::has(const std::string& name) const {
    return names.find(name) != names.end();
}

std::unique_ptr<Atlas> AtlasBuilder::build(uint extrusion, bool prepare, uint maxResolution) {
    if (maxResolution == 0) {
        maxResolution = Texture::MAX_RESOLUTION;
    }
    auto sizes = std::make_unique<uint[]>(entries.size() * 2);
    uint index = 0;
    for (auto& entry : entries) {
        auto image = entry.image;
        sizes[index++] = image->getWidth();
        sizes[index++] = image->getHeight();
    }
    LMPacker packer(sizes.get(), entries.size()*2);
    sizes.reset(nullptr);

    uint width = 32;
    uint height = 32;
    while (!packer.buildCompact(width, height, extrusion)) {
        if (width > height) {
            height *= 2;
        } else {
            width *= 2;
        }
        if (width > maxResolution || height > maxResolution) {
            throw std::runtime_error(
                "max atlas resolution "+std::to_string(maxResolution)+" exceeded"
            );
        }
    }

    auto canvas = std::make_unique<ImageData>(ImageFormat::rgba8888, width, height);
    std::unordered_map<std::string, UVRegion> regions;
    std::vector<rectangle> rects = packer.getResult();
    for (uint i = 0; i < entries.size(); i++) {
        const rectangle& rect = rects[i];
        const atlasentry& entry = entries[rect.idx];
        uint x = rect.x;
        uint y = rect.y;
        uint w = rect.width;
        uint h = rect.height;
        canvas->blit(*entry.image, rect.x, rect.y);
        for (uint j = 0; j < extrusion; j++) {
            canvas->extrude(x - j, y - j, w + j*2, h + j*2);
        }
        float unitX = 1.0f / width;
        float unitY = 1.0f / height;
        regions[entry.name] = UVRegion(
            unitX * x, unitY * y, unitX * (x + w), unitY * (y + h)
        );
    }
    return std::make_unique<Atlas>(std::move(canvas), regions, prepare);
}
