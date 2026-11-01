#include "ContentBuilder.hpp"

ContentBuilder::~ContentBuilder() = default;

void ContentBuilder::add(std::unique_ptr<ContentPackRuntime> pack) {
    packs[pack->getId()] = std::move(pack);
}

BlockMaterial& ContentBuilder::createBlockMaterial(const std::string& id) {
    blockMaterials[id] = std::make_unique<BlockMaterial>();
    auto& material = *blockMaterials[id];
    material.name = id;
    return material;
}

std::unique_ptr<Content> ContentBuilder::build() {
    std::vector<Block*> blockDefsIndices;
    auto groups = std::make_unique<DrawGroups>();
    for (const std::string& name : blocks.names) {
        Block& def = *blocks.defs[name];

        // Generating runtime info
        def.rt.id = blockDefsIndices.size();
        def.rt.emissive = *reinterpret_cast<uint32_t*>(def.emission);
        for (const auto& tag : def.tags) {
            def.rt.tags.insert(tags.add(tag));
        }

        auto isSolid = [](const Variant& variant) {
            return variant.model.type == BlockModelType::BLOCK ||
                   variant.culling == CullingMode::OPTIONAL;
        };

        auto prepareHitboxes = [&def](Variant& variant) {
            if (def.rotatable) {
                for (uint i = 0; i < BlockRotProfile::MAX_COUNT; i++) {
                    auto& rotated = variant.rt.hitboxes[i];
                    rotated.clear();
                    rotated.reserve(variant.hitboxes.size());
                    for (AABB aabb : variant.hitboxes) {
                        def.rotations.variants[i].transform(aabb);
                        aabb.fix();
                        rotated.push_back(aabb);
                    }
                }
            } else {
                variant.rt.hitboxes[0] = variant.hitboxes;
            }
        };

        const float EPSILON = 0.01f;
        auto isFullHitbox = [&def, EPSILON](const Variant& variant) {
            return !variant.hitboxes.empty() &&
                   glm::i8vec3(variant.hitboxes[0].size() + EPSILON) == def.size;
        };

        bool fullHitboxes;
        if (def.variants) {
            fullHitboxes = true;
            for (auto& variant : def.variants->variants) {
                variant.rt.solid = isSolid(variant) || def.explictlySolid;
                prepareHitboxes(variant);
                fullHitboxes = fullHitboxes && isFullHitbox(variant);
            }
            def.defaults = def.variants->variants.at(0);
        } else {
            def.defaults.rt.solid = isSolid(def.defaults) || def.explictlySolid;
            prepareHitboxes(def.defaults);
            fullHitboxes = isFullHitbox(def.defaults);
        }
        def.rt.solid = def.defaults.rt.solid && def.obstacle && fullHitboxes;
        def.rt.extended = def.size.x > 1 || def.size.y > 1 || def.size.z > 1;

        if (def.material.empty()) {
            defaults.at("block-material").get(def.material);
        }

        blockDefsIndices.push_back(&def);
        groups->insert(def.defaults.drawGroup); // FIXME: variants
    }

    std::vector<ItemDef*> itemDefsIndices;
    for (const std::string& name : items.names) {
        ItemDef& def = *items.defs[name];

        // Generating runtime info
        def.rt.id = itemDefsIndices.size();
        def.rt.emissive = *reinterpret_cast<uint32_t*>(def.emission);
        itemDefsIndices.push_back(&def);
    }

    std::vector<EntityDef*> entityDefsIndices;
    for (const std::string& name : entities.names) {
        EntityDef& def = *entities.defs[name];

        // Generating runtime info
        def.rt.id = entityDefsIndices.size();
        entityDefsIndices.push_back(&def);
    }

    auto content = std::make_unique<Content>(
        std::make_unique<ContentIndices>(
            blockDefsIndices, itemDefsIndices, entityDefsIndices
        ),
        std::move(groups),
        blocks.build(),
        items.build(),
        entities.build(),
        generators.build(),
        std::move(packs),
        std::move(blockMaterials),
        std::move(resourceIndices),
        std::move(defaults),
        std::move(tags.map)
    );

    // Now, it's time to resolve foreign keys
    for (Block* def : blockDefsIndices) {
        def->rt.pickingItem = content->items.require(def->pickingItem).rt.id;
        def->rt.surfaceReplacement = content->blocks.require(def->surfaceReplacement).rt.id;
        if (def->properties == nullptr) {
            def->properties = dv::object();
        }
        def->properties["name"] = def->name;
        def->properties["script-file"] = def->scriptFile;
    }

    for (ItemDef* def : itemDefsIndices) {
        def->rt.placingBlock = content->blocks.require(def->placingBlock).rt.id;
        if (def->properties == nullptr) {
            def->properties = dv::object();
        }
        def->properties["name"] = def->name;
        def->properties["script-file"] = def->scriptFile;
    }

    for (EntityDef* def : entityDefsIndices) {
        if (def->properties == nullptr) {
            def->properties = dv::object();
        }
        def->properties["name"] = def->name;
    }

    for (auto& [name, def] : content->generators.getDefs()) {
        def->prepare(content.get());
    }

    return content;
}
