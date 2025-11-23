#define VC_ENABLE_REFLECTION
#include "ContentUnitLoader.hpp"
#include "ContentLoadingCommons.hpp"

#include "../ContentBuilder.hpp"
#include "coders/json.hpp"
#include "core_defs.hpp"
#include "data/dv.hpp"
#include "data/StructLayout.hpp"
#include "debug/Logger.hpp"
#include "io/io.hpp"
#include "presets/ParticlesPreset.hpp"
#include "util/stringutil.hpp"
#include "voxels/Block.hpp"

using namespace data;

static debug::Logger logger("block-content-loader");

static void perform_user_block_fields(
    const std::string& blockName, StructLayout& layout
) {
    if (layout.size() > MAX_USER_BLOCK_FIELDS_SIZE) {
        throw std::runtime_error(
            util::quote(blockName) + 
            " fields total size exceeds limit (" + 
            std::to_string(layout.size()) + "/" +
            std::to_string(MAX_USER_BLOCK_FIELDS_SIZE) + ")");
    }
    for (const auto& field : layout) {
        if (field.name.at(0) == '.') {
            throw std::runtime_error(
                util::quote(blockName) + " field " + field.name + 
                ": user field may not start with '.'");
        }
    }

    std::vector<Field> fields;
    fields.insert(fields.end(), layout.begin(), layout.end());
    // add built-in fields here
    layout = StructLayout::create(fields);
}

static void load_variant(
    Variant& variant, const dv::value& root, const std::string& name
) {
    // block texturing
    if (root.has("texture")) {
        const auto& texture = root["texture"].asString();
        for (uint i = 0; i < 6; i++) {
            variant.textureFaces[i] = texture;
        }
    } else if (root.has("texture-faces")) {
        const auto& texarr = root["texture-faces"];
        for (uint i = 0; i < 6; i++) {
            variant.textureFaces[i] = texarr[i].asString();
        }
    }

    // block model
    auto& model = variant.model;
    std::string modelTypeName = BlockModelTypeMeta.getNameString(model.type);
    root.at("model").get(modelTypeName);
    root.at("model-name").get(model.name);
    if (BlockModelTypeMeta.getItem(modelTypeName, model.type)) {
        if (model.type == BlockModelType::CUSTOM && model.customRaw == nullptr) {
            if (root.has("model-primitives")) {
                model.customRaw = root["model-primitives"];
            } else if (model.name.empty()) {
                throw std::runtime_error(
                    name + ": no 'model-primitives' or 'model-name' found"
                );
            }
        }
    } else if (!modelTypeName.empty()) {
        logger.error() << "unknown model: " << modelTypeName;
        model.type = BlockModelType::NONE;
    }
    std::string cullingModeName = CullingModeMeta.getNameString(variant.culling);
    root.at("culling").get(cullingModeName);
    if (!CullingModeMeta.getItem(cullingModeName, variant.culling)) {
        logger.error() << "unknown culling mode: " << cullingModeName;
    }
    root.at("draw-group").get(variant.drawGroup);
}

template<> void ContentUnitLoader<Block>::loadUnit(
    Block& def, const std::string& name, const io::path& file
) {
    auto root = io::read_json(file);
    process_properties(def, name, root);
    process_tags(def, root);

    if (root.has("parent")) {
        const auto& parentName = root["parent"].asString();
        auto parentDef = builder.get(parentName);
        if (parentDef == nullptr) {
            throw std::runtime_error(
                "Failed to find parent(" + parentName + ") for " + name
            );
        }
        parentDef->cloneTo(def);
    }

    root.at("caption").get(def.caption);

    load_variant(def.defaults, root, name);

    if (root.has("state-based")) {
        const auto& stateBased = root["state-based"];
        if (stateBased.has("variants")) {
            const auto& variants = stateBased["variants"];

            int offset = 0;
            int bitsCount = 4;
            stateBased.at("offset").get(offset);
            stateBased.at("bits").get(bitsCount);
            if (offset < 0 || bitsCount <= 0 || offset + bitsCount > 8) {
                throw std::runtime_error("Invalid state-based bits configuration");
            }

            def.variants = std::make_unique<Variants>();
            def.variants->offset = 0;
            def.variants->mask = 0xF;
            def.variants->variants.push_back(def.defaults);
            for (int i = 0; i < variants.size(); i++) {
                Variant variant = def.defaults;
                load_variant(variant, variants[i], name);
                def.variants->variants.push_back(variant);
            }
            while (def.variants->variants.size() < BLOCK_MAX_VARIANTS) {
                def.variants->variants.push_back(def.defaults);
            }
        }
    }

    root.at("material").get(def.material);

    // rotation profile
    std::string profile = def.rotations.name;
    root.at("rotation").get(profile);

    def.rotatable = profile != "none";
    if (profile == BlockRotProfile::PIPE_NAME) {
        def.rotations = BlockRotProfile::PIPE;
    } else if (profile == BlockRotProfile::PANE_NAME) {
        def.rotations = BlockRotProfile::PANE;
    } else if (profile == BlockRotProfile::STAIRS_NAME) {
        def.rotations = BlockRotProfile::STAIRS;
    } else if (profile != "none") {
        logger.error() << "unknown rotation profile " << profile;
        def.rotatable = false;
    }

    // block hitbox AABB [x, y, z, width, height, depth]
    if (auto found = root.at("hitboxes")) {
        const auto& boxarr = *found;
        def.hitboxes.resize(boxarr.size());
        for (uint i = 0; i < boxarr.size(); i++) {
            const auto& box = boxarr[i];
            auto& hitboxesIndex = def.hitboxes[i];
            hitboxesIndex.a = glm::vec3(
                box[0].asNumber(), box[1].asNumber(), box[2].asNumber()
            );
            hitboxesIndex.b = glm::vec3(
                box[3].asNumber(), box[4].asNumber(), box[5].asNumber()
            );
            hitboxesIndex.b += hitboxesIndex.a;
        }
    } else if (auto found = root.at("hitbox")) {
        const auto& box = *found;
        AABB aabb;
        aabb.a = glm::vec3(
            box[0].asNumber(), box[1].asNumber(), box[2].asNumber()
        );
        aabb.b = glm::vec3(
            box[3].asNumber(), box[4].asNumber(), box[5].asNumber()
        );
        aabb.b += aabb.a;
        def.hitboxes = {aabb};
    }

    // block light emission [r, g, b] where r,g,b in range [0..15]
    if (auto found = root.at("emission")) {
        const auto& emissionarr = *found;
        for (size_t i = 0; i < 3; i++) {
            def.emission[i] = glm::clamp(emissionarr[i].asInteger(), static_cast<integer_t>(0), static_cast<integer_t>(15));
        }
    }

    // block size
    if (auto found = root.at("size")) {
        const auto& sizearr = *found;
        def.size.x = sizearr[0].asInteger();
        def.size.y = sizearr[1].asInteger();
        def.size.z = sizearr[2].asInteger();
        if (def.size.x < 1 || def.size.y < 1 || def.size.z < 1) {
            throw std::runtime_error(
                "block " + util::quote(def.name) + ": invalid block size"
            );
        }
        
        // should variant modify hitbox?
        if (def.defaults.model.type == BlockModelType::BLOCK &&
            (def.size.x != 1 || def.size.y != 1 || def.size.z != 1)) {
            def.defaults.model.type = BlockModelType::AABB;
            def.hitboxes = {AABB(def.size)};
        }
    }

    // primitive properties
    root.at("obstacle").get(def.obstacle);
    root.at("replaceable").get(def.replaceable);
    root.at("light-passing").get(def.lightPassing);
    root.at("sky-light-passing").get(def.skyLightPassing);
    root.at("shadeless").get(def.shadeless);
    root.at("ambient-occlusion").get(def.ambientOcclusion);
    root.at("breakable").get(def.breakable);
    root.at("selectable").get(def.selectable);
    root.at("grounded").get(def.grounded);
    root.at("hidden").get(def.hidden);
    root.at("picking-item").get(def.pickingItem);
    root.at("surface-replacement").get(def.surfaceReplacement);
    root.at("script-name").get(def.scriptName);
    root.at("ui-layout").get(def.uiLayout);
    root.at("inventory-size").get(def.inventorySize);
    root.at("tick-interval").get(def.tickInterval);
    root.at("overlay-texture").get(def.overlayTexture);
    root.at("translucent").get(def.translucent);

    if (root.has("fields")) {
        def.dataStruct = std::make_unique<StructLayout>();
        def.dataStruct->deserialize(root["fields"]);

        perform_user_block_fields(def.name, *def.dataStruct);
    }

    if (root.has("particles")) {
        def.particles = std::make_unique<ParticlesPreset>();
        def.particles->deserialize(root["particles"]);
    }

    if (def.tickInterval == 0) {
        def.tickInterval = 1;
    }

    if (def.hidden && def.pickingItem == def.name + BLOCK_ITEM_SUFFIX) {
        def.pickingItem = CORE_EMPTY;
    }
    def.scriptFile = pack.id + ":scripts/" + def.scriptName + ".lua";
}
