#include "ModelsGenerator.hpp"

#include "SpriteExtrusion.hpp"
#include "assets/Assets.hpp"
#include "assets/assets_util.hpp"
#include "graphics/core/Atlas.hpp"
#include "graphics/core/ImageData.hpp"
#include "items/ItemDef.hpp"
#include "voxels/Block.hpp"
#include "content/Content.hpp"
#include "debug/Logger.hpp"
#include "core_defs.hpp"

#include <cmath>
#include <optional>

static debug::Logger logger("models-generator");

static void configure_textures(
    model::Model& model,
    const Assets& assets,
    const std::array<std::string, 6>& textureFaces
) {
    for (auto& mesh : model.meshes) {
        auto& texture = mesh.texture;
        if (texture.empty() || texture.at(0) != '$') {
            continue;
        }
        try {
            int index = std::stoi(texture.substr(1));
            texture = "blocks:" + textureFaces.at(index);
        } catch (const std::invalid_argument& err) {
        } catch (const std::runtime_error& err) {
            logger.error() << err.what();
        }
    }
}

/// The square a flat item is drawn in, in blocks. Where an item sits in a
/// hand and how large it looks lying on the ground are both measured off
/// this, so it is the size the model this replaced was drawn at.
inline constexpr float SPRITE_SIZE = 0.565f;
/// And that model sat a hair above the middle of its own square. Kept, so
/// that nothing an item hangs off has to move.
inline constexpr float SPRITE_Y_OFFSET = 0.0107f;
/// Below this an alpha channel is a hole rather than a faint pixel. The
/// halfway point the alpha clip in the shader uses, so the rim ends up
/// where the sprite visibly ends.
inline constexpr ubyte SPRITE_ALPHA_CLIP = 128;

/// @brief The pixels behind a texture name, and which of them are the sprite.
///
/// An atlas keeps its raster after handing one to the GPU, which is the
/// only reason any of this can be worked out without reading a texture
/// back off it.
struct SpritePixels {
    const ImageData* image;
    int x, y, width, height;
};

static std::optional<SpritePixels> find_sprite_pixels(
    const Assets& assets, const std::string& name
) {
    size_t sep = name.find(':');
    if (sep == std::string::npos || sep + 1 >= name.length()) {
        return std::nullopt;
    }
    const auto* atlas = assets.get<Atlas>(name.substr(0, sep));
    if (atlas == nullptr) {
        return std::nullopt;
    }
    const auto* image = atlas->getImage();
    auto region = atlas->getIf(name.substr(sep + 1));
    if (image == nullptr || !region.has_value()) {
        return std::nullopt;
    }
    auto to_pixels = [](float v, uint size) {
        return static_cast<int>(std::lround(v * static_cast<float>(size)));
    };
    int x = to_pixels(region->u1, image->getWidth());
    int y = to_pixels(region->v1, image->getHeight());
    int width = to_pixels(region->u2, image->getWidth()) - x;
    int height = to_pixels(region->v2, image->getHeight()) - y;
    if (width <= 0 || height <= 0) {
        return std::nullopt;
    }
    return SpritePixels {image, x, y, width, height};
}

/// A picture with no alpha channel has no holes in it, which is the answer
/// this gives rather than a special case somewhere further down.
static std::vector<bool> read_alpha_mask(const SpritePixels& sprite) {
    const auto& image = *sprite.image;
    std::vector<bool> opaque(
        static_cast<size_t>(sprite.width) * sprite.height, true
    );
    if (image.getFormat() != ImageFormat::RGBA8888) {
        return opaque;
    }
    const ubyte* data = image.getData();
    for (int y = 0; y < sprite.height; y++) {
        for (int x = 0; x < sprite.width; x++) {
            size_t at =
                ((static_cast<size_t>(sprite.y + y) * image.getWidth()) +
                 sprite.x + x) * 4;
            opaque[static_cast<size_t>(y) * sprite.width + x] =
                data[at + 3] >= SPRITE_ALPHA_CLIP;
        }
    }
    return opaque;
}

/// @brief A sprite given the thickness of one of its own pixels.
///
/// The model this replaced was forty-two copies of the sprite stacked a
/// fraction apart with no sides at all: solid enough from the front, and
/// edge-on nothing but coplanar quads fighting over the depth buffer. This
/// is a front, a back, and a rim that follows the shape the sprite
/// actually is, so from the side an item is its own silhouette.
static model::Model create_flat_model(
    const std::string& texture, const Assets& assets
) {
    auto sprite = find_sprite_pixels(assets, texture);
    // Without the raster there is nothing to trace. A plain slab is still
    // a solid of the right size; it is only square where it should have
    // been the shape of the drawing.
    int width = 1;
    int height = 1;
    std::vector<bool> opaque {true};
    if (sprite.has_value()) {
        width = sprite->width;
        height = sprite->height;
        opaque = read_alpha_mask(*sprite);
    }
    const float pixelX = SPRITE_SIZE / width;
    const float pixelY = SPRITE_SIZE / height;
    // One pixel thick, so a sprite is as deep as its own pixels are wide
    // and reads as a slab of them rather than a sheet.
    const float halfThickness = pixelX * 0.5f;
    const float left = -SPRITE_SIZE * 0.5f;
    const float bottom = -SPRITE_SIZE * 0.5f + SPRITE_Y_OFFSET;

    model::Model model;
    auto& mesh = model.addMesh(texture);
    const glm::vec3 centre {0.0f, SPRITE_Y_OFFSET, 0.0f};
    const glm::vec3 halfX {SPRITE_SIZE * 0.5f, 0.0f, 0.0f};
    const glm::vec3 halfY {0.0f, SPRITE_SIZE * 0.5f, 0.0f};
    const glm::vec3 depth {0.0f, 0.0f, halfThickness};
    // Whole, both of them: the transparent pixels are drawn and thrown
    // away by the alpha clip exactly as they always were. What the rim
    // adds below is the part a flat sprite never had.
    mesh.addPlane(
        centre + depth, halfX, halfY, {0, 0, 1}, UVRegion(0, 0, 1, 1)
    );
    mesh.addPlane(
        centre - depth, -halfX, halfY, {0, 0, -1}, UVRegion(1, 0, 0, 1)
    );

    for (const auto& edge : find_sprite_edges(opaque, width)) {
        // A rim face takes the colour of the pixel that asked for it,
        // sampled down the middle of that pixel: the boundary between a
        // drawn pixel and the hole beside it is exactly where a sampler
        // cannot be trusted to pick the drawn one.
        if (edge.outwardY != 0) {
            const float y =
                bottom + (edge.line + (edge.outwardY > 0 ? 1 : 0)) * pixelY;
            const float x0 = left + edge.from * pixelX;
            const float x1 = left + (edge.to + 1) * pixelX;
            const float v = (edge.line + 0.5f) / height;
            mesh.addPlane(
                {(x0 + x1) * 0.5f, y, 0.0f},
                {(x1 - x0) * 0.5f, 0.0f, 0.0f},
                // right cross up has to come out the way the face looks,
                // or it is a face drawn from behind and culled away.
                {0.0f, 0.0f, -halfThickness * edge.outwardY},
                {0.0f, static_cast<float>(edge.outwardY), 0.0f},
                UVRegion(
                    (edge.from + 0.5f) / width, v, (edge.to + 0.5f) / width, v
                )
            );
        } else {
            const float x =
                left + (edge.line + (edge.outwardX > 0 ? 1 : 0)) * pixelX;
            const float y0 = bottom + edge.from * pixelY;
            const float y1 = bottom + (edge.to + 1) * pixelY;
            const float u = (edge.line + 0.5f) / width;
            mesh.addPlane(
                {x, (y0 + y1) * 0.5f, 0.0f},
                {0.0f, 0.0f, -halfThickness * edge.outwardX},
                {0.0f, (y1 - y0) * 0.5f, 0.0f},
                {static_cast<float>(edge.outwardX), 0.0f, 0.0f},
                UVRegion(
                    u, (edge.from + 0.5f) / height, u, (edge.to + 0.5f) / height
                )
            );
        }
    }
    return model;
}

static inline UVRegion get_region_for(
    const std::string& texture, const Assets& assets
) {
    auto texreg = util::get_texture_region(assets, "blocks:" + texture, "");
    return texreg.region;
}

void ModelsGenerator::prepareModel(
    Assets& assets, const Block& def, Variant& variant, uint8_t variantId
) {
    BlockModel& blockModel = variant.model;
    if (blockModel.type == BlockModelType::CUSTOM) {
        std::string modelName = def.name + ".model" + (variantId == 0 ? "" : "$" + std::to_string(variantId));
        if (blockModel.name.empty()) {
            assets.store(
                std::make_unique<model::Model>(
                    loadCustomBlockModel(
                        blockModel.customRaw, assets, !def.shadeless
                    )
                ),
                modelName
            );
            blockModel.name = modelName;
        } else {
            auto srcModel = assets.get<model::Model>(blockModel.name);
            if (srcModel) {
                bool defaultAssigned = variant.textureFaces[0] != TEXTURE_NOTFOUND;
                auto model = std::make_unique<model::Model>(*srcModel);
                for (auto& mesh : model->meshes) {
                    if (mesh.texture.length() && mesh.texture[0] == '$') {
                        int index = std::stoll(mesh.texture.substr(1));
                        mesh.texture = "blocks:" + variant.textureFaces[index];
                    } else if (!defaultAssigned && !mesh.texture.empty()) {
                        size_t sepPos = mesh.texture.find(':');
                        if (sepPos == std::string::npos)
                            continue;
                        variant.textureFaces[0] = mesh.texture.substr(sepPos + 1);
                        defaultAssigned = true;
                    }
                }
                blockModel.name = modelName;
                assets.store(std::move(model), blockModel.name);
            }
        }
    }
}

void ModelsGenerator::prepare(Content& content, Assets& assets) {
    for (auto& [name, def] : content.blocks.getDefs()) {
        prepareModel(assets, *def, def->defaults, 0);
        if (def->variants) {
            auto& variants = def->variants->variants;
            for (int i = 1; i < variants.size(); i++) {
                prepareModel(assets, *def, variants[i], i);
            }
        }
    }
    for (auto& [name, def] : content.items.getDefs()) {
        assets.store(
            std::make_unique<model::Model>(
                generate(*def, content, assets)
            ),
            name + ".model"
        );
    }
}

model::Model ModelsGenerator::fromCustom(
    const Assets& assets,
    const std::vector<AABB>& modelBoxes,
    const std::vector<std::string>& modelTextures,
    const std::vector<glm::vec3>& points,
    bool lighting
) {
    auto model = model::Model();
    for (size_t i = 0; i < modelBoxes.size(); i++) {
        auto& mesh = model.addMesh("blocks:");
        mesh.shading = lighting;
        UVRegion boxtexfaces[6] = {
            get_region_for(modelTextures[i * 6 + 5], assets),
            get_region_for(modelTextures[i * 6 + 4], assets),
            get_region_for(modelTextures[i * 6 + 3], assets),
            get_region_for(modelTextures[i * 6 + 2], assets),
            get_region_for(modelTextures[i * 6 + 1], assets),
            get_region_for(modelTextures[i * 6 + 0], assets)
        };
        boxtexfaces[2].scale(glm::vec2(-1));
        boxtexfaces[5].scale(glm::vec2(-1, 1));

        bool enabled[6] {1,1,1,1,1,1};
        mesh.addBox(
            modelBoxes[i].center(),
            modelBoxes[i].size() * 0.5f,
            boxtexfaces,
            enabled
        );
    }
    for (size_t i = 0; i < points.size() / 4; i++) {
        auto texture = modelTextures[modelBoxes.size() * 6 + i];

        const glm::vec3& v0 = points[i * 4];
        const glm::vec3& v1 = points[i * 4 + 1];
        const glm::vec3& v2 = points[i * 4 + 2];
        const glm::vec3& v3 = points[i * 4 + 3];

        glm::vec3 edge1 = v1 - v0;
        glm::vec3 edge2 = v2 - v0;

        glm::vec3 norm = glm::cross(edge1, edge2);
        norm = glm::normalize(norm);

        auto& mesh = model.addMesh(texture);
        mesh.shading = lighting;

        auto reg = get_region_for(texture, assets);
        mesh.vertices.push_back({v0, glm::vec2(reg.u1, reg.v1), norm});
        mesh.vertices.push_back({v1, glm::vec2(reg.u2, reg.v1), norm});
        mesh.vertices.push_back({v2, glm::vec2(reg.u2, reg.v2), norm});
        mesh.vertices.push_back({v0, glm::vec2(reg.u1, reg.v1), norm});
        mesh.vertices.push_back({v2, glm::vec2(reg.u2, reg.v2), norm});
        mesh.vertices.push_back({v3, glm::vec2(reg.u1, reg.v2), norm});
    }
    return model;
}

model::Model ModelsGenerator::generate(
    const ItemDef& def, const Content& content, const Assets& assets
) {
    if (def.iconType == ItemIconType::BLOCK) {
        auto model = assets.require<model::Model>("block");
        const auto& blockDef = content.blocks.require(def.icon);
        const auto& variant = blockDef.defaults;
        const auto& blockModel = variant.model;
        if (blockModel.type == BlockModelType::XSPRITE) {
            return create_flat_model(
                "blocks:" + blockDef.defaults.textureFaces.at(0), assets
            );
        } else if (blockModel.type == BlockModelType::CUSTOM) {
            model = assets.require<model::Model>(blockModel.name);
            for (auto& mesh : model.meshes) {
                mesh.scale(glm::vec3(0.2f));
            }
            return model;
        }
        for (auto& mesh : model.meshes) {
            mesh.shading = !blockDef.shadeless;
            switch (blockModel.type) {
                case BlockModelType::AABB: {
                    glm::vec3 size = blockDef.hitboxes.at(0).size();
                    float m = glm::max(size.x, glm::max(size.y, size.z));
                    m = glm::min(1.0f, m);
                    mesh.scale(size / m);
                    break;
                } default:
                    break;
            }
            mesh.scale(glm::vec3(0.2f));
        }
        configure_textures(model, assets, blockDef.defaults.textureFaces);
        return model;
    } else if (def.iconType == ItemIconType::SPRITE) {
        return create_flat_model(def.icon, assets);
    } else {
        return model::Model();
    }
}

model::Model ModelsGenerator::loadCustomBlockModel(
    const dv::value& primitives, const Assets& assets, bool lighting
) {
    std::vector<AABB> modelBoxes;
    std::vector<std::string> modelTextures;
    std::vector<glm::vec3> modelExtraPoints;

    if (primitives.has("aabbs")) {
        const auto& modelboxes = primitives["aabbs"];
        for (uint i = 0; i < modelboxes.size(); i++) {
            // Parse aabb
            const auto& boxarr = modelboxes[i];
            AABB modelbox;
            modelbox.a = glm::vec3(
                boxarr[0].asNumber(), boxarr[1].asNumber(), boxarr[2].asNumber()
            );
            modelbox.b = glm::vec3(
                boxarr[3].asNumber(), boxarr[4].asNumber(), boxarr[5].asNumber()
            );
            modelbox.b += modelbox.a;
            modelBoxes.push_back(modelbox);

            if (boxarr.size() == 7) {
                for (uint j = 6; j < 12; j++) {
                    modelTextures.emplace_back(boxarr[6].asString());
                }
            } else if (boxarr.size() == 12) {
                for (uint j = 6; j < 12; j++) {
                    modelTextures.emplace_back(boxarr[j].asString());
                }
            } else {
                for (uint j = 6; j < 12; j++) {
                    modelTextures.emplace_back("notfound");
                }
            }
        }
    }
    if (primitives.has("tetragons")) {
        const auto& modeltetragons = primitives["tetragons"];
        for (uint i = 0; i < modeltetragons.size(); i++) {
            // Parse tetragon to points
            const auto& tgonobj = modeltetragons[i];
            glm::vec3 p1(
                tgonobj[0].asNumber(), tgonobj[1].asNumber(), tgonobj[2].asNumber()
            );
            glm::vec3 xw(
                tgonobj[3].asNumber(), tgonobj[4].asNumber(), tgonobj[5].asNumber()
            );
            glm::vec3 yh(
                tgonobj[6].asNumber(), tgonobj[7].asNumber(), tgonobj[8].asNumber()
            );
            modelExtraPoints.push_back(p1);
            modelExtraPoints.push_back(p1 + xw);
            modelExtraPoints.push_back(p1 + xw + yh);
            modelExtraPoints.push_back(p1 + yh);

            modelTextures.emplace_back(tgonobj[9].asString());
        }
    }
    return fromCustom(
        assets, modelBoxes, modelTextures, modelExtraPoints, lighting
    );
}
