#include "ModelsGenerator.hpp"

#include "assets/Assets.hpp"
#include "assets/assets_util.hpp"
#include "items/ItemDef.hpp"
#include "voxels/Block.hpp"
#include "content/Content.hpp"
#include "debug/Logger.hpp"

static debug::Logger logger("models-generator");

static void configure_textures(
    model::Model& model,
    const Block& blockDef,
    const Assets& assets
) {
    for (auto& mesh : model.meshes) {
        auto& texture = mesh.texture;
        if (texture.empty() || texture.at(0) != '$') {
            continue;
        }
        try {
            int index = std::stoi(texture.substr(1));
            texture = "blocks:"+blockDef.textureFaces.at(index);
        } catch (const std::invalid_argument& err) {
        } catch (const std::runtime_error& err) {
            logger.error() << err.what();
        }
    }
}

static model::Model create_flat_model(
    const std::string& texture, const Assets& assets
) {
    auto model = assets.require<model::Model>("drop-item");
    for (auto& mesh : model.meshes) {
        if (mesh.texture == "$0") {
            mesh.texture = texture;
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

void ModelsGenerator::prepare(Content& content, Assets& assets) {
    for (auto& [name, def] : content.blocks.getDefs()) {
        if (def->model.type == BlockModelType::CUSTOM) {
            if (def->model.name.empty()) {
                assets.store(
                    std::make_unique<model::Model>(
                        loadCustomBlockModel(
                            def->model.customRaw, assets, !def->shadeless
                        )
                    ),
                    name + ".model"
                );
                def->model.name = def->name + ".model";
            } else {
                auto srcModel = assets.get<model::Model>(def->model.name);
                if (srcModel) {
                    auto model = std::make_unique<model::Model>(*srcModel);
                    for (auto& mesh : model->meshes) {
                        if (mesh.texture.length() && mesh.texture[0] == '$') {
                            int index = std::stoll(mesh.texture.substr(1));
                            mesh.texture = "blocks:" + def->textureFaces[index];
                        }
                    }
                    def->model.name = name + ".model";
                    assets.store(std::move(model), def->model.name);
                }
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
        mesh.lighting = lighting;
        const UVRegion boxtexfaces[6] = {
            get_region_for(modelTextures[i * 6 + 5], assets),
            get_region_for(modelTextures[i * 6 + 4], assets),
            get_region_for(modelTextures[i * 6 + 3], assets),
            get_region_for(modelTextures[i * 6 + 2], assets),
            get_region_for(modelTextures[i * 6 + 1], assets),
            get_region_for(modelTextures[i * 6 + 0], assets)
        };
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
        mesh.lighting = lighting;

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
        if (blockDef.model.type == BlockModelType::XSPRITE) {
            return create_flat_model(
                "blocks:" + blockDef.textureFaces.at(0), assets
            );
        } else if (blockDef.model.type == BlockModelType::CUSTOM) {
            model = assets.require<model::Model>(blockDef.model.name);
            for (auto& mesh : model.meshes) {
                mesh.scale(glm::vec3(0.2f));
            }
            return model;
        }
        for (auto& mesh : model.meshes) {
            mesh.lighting = !blockDef.shadeless;
            switch (blockDef.model.type) {
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
        configure_textures(model, blockDef, assets);
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
