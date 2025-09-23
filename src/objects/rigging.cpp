#include "rigging.hpp"

#include "assets/Assets.hpp"
#include "coders/json.hpp"
#include "data/dv_util.hpp"
#include "graphics/commons/Model.hpp"
#include "graphics/render/ModelBatch.hpp"

#include <glm/ext/matrix_transform.hpp>
#include <glm/gtx/matrix_decompose.hpp>

using namespace rigging;

void ModelReference::refresh(const Assets& assets) {
    if (updateFlag) {
        model = assets.get<model::Model>(name);
        updateFlag = false;
    }
}

Bone::Bone(
    size_t index,
    std::string name,
    std::string model,
    std::vector<std::unique_ptr<Bone>> bones,
    const glm::vec3& offset
)
    : index(index),
      name(std::move(name)),
      bones(std::move(bones)),
      offset(offset),
      model({model, nullptr, true}) {
}

void Bone::setModel(const std::string& name) {
    if (model.name == name) {
        return;
    }
    model = {name, nullptr, true};
}

Skeleton::Skeleton(const SkeletonConfig* config)
    : config(config),
      pose(config->getBones().size()),
      calculated(config->getBones().size()),
      flags(config->getBones().size()),
      textures(),
      modelOverrides(config->getBones().size()),
      visible(true) {
    const auto& bones = config->getBones();
    for (size_t i = 0; i < bones.size(); i++) {
        flags[i].visible = true;
    }
}

dv::value Skeleton::serialize(bool saveTextures, bool savePose) const {
    auto root = dv::object();
    if (saveTextures) {
        auto& map = root.object("textures");
        for (auto& [slot, texture] : textures) {
            map[slot] = texture;
        }
    }
    if (savePose) {
        auto& list = root.list("pose");
        for (auto& mat : pose.matrices) {
            list.add(dv::to_value(mat));
        }
    }
    return root;
}

void Skeleton::deserialize(const dv::value& root) {
    if (auto found = root.at("textures")) {
        auto& texturesmap = *found;
        for (auto& [slot, _] : texturesmap.asObject()) {
            texturesmap.at(slot).get(textures[slot]);
        }
    }
    if (auto found = root.at("pose")) {
        auto& posearr = *found;
        auto& matrices = pose.matrices;
        for (size_t i = 0; i < std::min(matrices.size(), posearr.size()); i++) {
            dv::get_mat(posearr[i], pose.matrices[i]);
        }
    }
}

static void get_all_nodes(std::vector<Bone*>& nodes, Bone* node) {
    nodes[node->getIndex()] = node;
    for (auto& subnode : node->getSubnodes()) {
        get_all_nodes(nodes, subnode.get());
    }
}

SkeletonConfig::SkeletonConfig(
    const std::string& name, std::unique_ptr<Bone> root, size_t nodesCount
)
    : name(name), root(std::move(root)), nodes(nodesCount) {
    get_all_nodes(nodes, this->root.get());
}

size_t SkeletonConfig::update(
    size_t index, Skeleton& skeleton, Bone* node, const glm::mat4& matrix
) const {
    auto boneMatrix = skeleton.pose.matrices[index];
    auto boneOffset = node->getOffset();
    glm::mat4 baseMatrix(1.0f);
    if (glm::length2(boneOffset) > 0.0f) {
        baseMatrix = glm::translate(glm::mat4(1.0f), boneOffset);
    }
    skeleton.calculated.matrices[index] = matrix * baseMatrix * boneMatrix;
    size_t count = 1;
    for (auto& subnode : node->getSubnodes()) {
        count += update(
            index + count,
            skeleton,
            subnode.get(),
            skeleton.calculated.matrices[index]
        );
    }
    return count;
}

void SkeletonConfig::update(
    Skeleton& skeleton, const glm::mat4& matrix, const glm::vec3& position
) const {
    if (skeleton.interpolation.isEnabled()) {
        const auto& interpolation = skeleton.interpolation;
        glm::vec3 scale, translation, skew;
        glm::quat rotation;
        glm::vec4 perspective;
        glm::decompose(matrix, scale, rotation, translation, skew, perspective);

        auto delta = interpolation.getCurrent() - position;
        auto interpolatedMatrix = glm::translate(matrix, delta);
        update(0, skeleton, root.get(), interpolatedMatrix);
    } else {
        update(0, skeleton, root.get(), matrix);
    }
}

void SkeletonConfig::render(
    const Assets& assets,
    ModelBatch& batch,
    Skeleton& skeleton,
    const glm::mat4& matrix,
    const glm::vec3& position
) const {
    update(skeleton, matrix, position);

    if (!skeleton.visible) {
        return;
    }
    for (size_t i = 0; i < nodes.size(); i++) {
        auto* node = nodes[i];
        if (!skeleton.flags[i].visible) {
            continue;
        }
        node->model.refresh(assets);
        auto model = node->model.model;
        auto& modelOverride = skeleton.modelOverrides.at(i);
        if (modelOverride.updateFlag) {
            modelOverride.refresh(assets);
        }
        model = modelOverride.model ? modelOverride.model : model;
        if (model) {
            batch.draw(
                skeleton.calculated.matrices[i],
                skeleton.tint,
                model,
                &skeleton.textures
            );
        }
    }
}

Bone* SkeletonConfig::find(std::string_view str) const {
    for (size_t i = 0; i < nodes.size(); i++) {
        auto* node = nodes[i];
        if (node->getName() == str) {
            return node;
        }
    }
    return nullptr;
}

static std::tuple<size_t, std::unique_ptr<Bone>> read_node(
    const dv::value& root, size_t index
) {
    std::string name;
    std::string model;
    root.at("name").get(name);
    root.at("model").get(model);
    glm::vec3 offset(0.0f);
    dv::get_vec(root, "offset", offset);

    std::vector<std::unique_ptr<Bone>> bones;
    size_t count = 1;
    if (auto found = root.at("nodes")) {
        const auto& nodesList = *found;
        for (const auto& map : nodesList) {
            auto [subcount, subNode] = read_node(map, index + count);
            count += subcount;
            bones.push_back(std::move(subNode));
        }
    }
    return {
        count,
        std::make_unique<Bone>(index, name, model, std::move(bones), offset)};
}

std::unique_ptr<SkeletonConfig> SkeletonConfig::parse(
    std::string_view src, std::string_view file, std::string_view name
) {
    auto root = json::parse(file, src);
    const auto& rootNodeMap = root["root"];
    auto [count, rootNode] = read_node(rootNodeMap, 0);
    return std::make_unique<SkeletonConfig>(
        std::string(name), std::move(rootNode), count
    );
}
