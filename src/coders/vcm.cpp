#include "vcm.hpp"

#include "xml.hpp"
#include "util/stringutil.hpp"
#include "io/io.hpp"

#include <vector>
#include <algorithm>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace vcm;
using namespace xml;
using namespace model;
using namespace rigging;

static int calc_offsets(
    const Bone& bone, std::vector<glm::vec3>& dst, int index, int depth, int parent
) {
    if (depth == 0) {
        dst[0] = bone.getOffset();
    } else {
        dst[index] = dst[parent] + bone.getOffset();
    }
    const auto& subBones = bone.getBones();

    int subIndex = index + 1;
    for (int i = 0; i < subBones.size(); i++) {
        subIndex += calc_offsets(*subBones[i], dst, subIndex, depth + 1, index);
    }
    return subIndex - index;
}

model::Model& VcmModel::squash() {
    std::vector<glm::vec3> fullOffsets(skeleton->getBones().size());
    calc_offsets(*skeleton->getRoot(), fullOffsets, 0, 0, 0);

    Model squashed;
    for (auto& [name, model] : parts) {
        if (auto bone = skeleton->find(name)) {
            model.translate(fullOffsets[bone->getIndex()]);
        } else {
            throw std::runtime_error("invalid state: bones/parts mismatch");
        }
        squashed.merge(std::move(model));
    }
    parts = { {"", std::move(squashed)} };
    skeleton.reset();
    return parts.at("");
}

model::Model VcmModel::squashed() const {
    return std::move(VcmModel {parts, std::nullopt}).squash();
}

static const std::unordered_map<std::string, int> side_indices {
    {"north", 0},
    {"south", 1},
    {"top", 2},
    {"bottom", 3},
    {"west", 4},
    {"east", 5},
};

static bool to_boolean(const xml::Attribute& attr) {
    return attr.getText() != "off";
}

class ModelBuilder {
public:
    ModelBuilder(Model& model) : model(model) {}

    void push(const glm::mat4& matrix) {
        matrices.push_back(matrix);
        calculateMatrix();
    }

    void pop() {
        matrices.pop_back();
        calculateMatrix();
    }

    const glm::mat4& getTransform() const {
        return combined;
    }

    void addBox(
        const std::string& texture,
        bool shading,
        const glm::vec3& pos,
        const glm::vec3& size,
        const UVRegion (&uvs)[6],
        const bool enabledSides[6]
    ) {
        auto& mesh = model.addMesh(texture, shading);
        mesh.addBox(pos, size, uvs, enabledSides, combined);
    }

    void addTriangle(
        const std::string& texture,
        bool shading,
        const glm::vec3& a,
        const glm::vec3& b,
        const glm::vec3& c,
        const glm::vec3& norm,
        const glm::vec2& uvA,
        const glm::vec2& uvB,
        const glm::vec2& uvC
    ) {
        auto& mesh = model.addMesh(texture, shading);
        mesh.addTriangle(
            combined * glm::vec4(a, 1.0f),
            combined * glm::vec4(b, 1.0f),
            combined * glm::vec4(c, 1.0f),
            norm,
            uvA,
            uvB,
            uvC
        );
    }

    void addRect(
        const std::string& texture,
        bool shading,
        const glm::vec3& pos,
        const glm::vec3& right,
        const glm::vec3& up,
        const glm::vec3& norm,
        const UVRegion& uv
    ) {
        auto& mesh = model.addMesh(texture, shading);
        mesh.addRect(pos, right, up, norm, uv, combined);
    }

    Model& getModel() {
        return model;
    }
private:
    void calculateMatrix() {
        combined = glm::mat4(1.0f);
        for (const auto& matrix : matrices) {
            combined *= matrix;
        }
    }

    Model& model;
    std::vector<glm::mat4> matrices;
    glm::mat4 combined {1.0f};
};

struct Context {
    VcmModel& vcmModel;
    std::vector<std::unique_ptr<Bone>>& bones;
    size_t& boneIndex;
};

static void perform_element(const xmlelement& root, ModelBuilder& builder, Context& ctx);

static void perform_rect(const xmlelement& root, ModelBuilder& builder) {
    auto from = root.attr("from").asVec3();
    auto right = root.attr("right").asVec3();
    auto up = root.attr("up").asVec3();
    bool shading = true;

    right *= -1;
    from -= right;

    UVRegion region {};
    if (root.has("region")) {
        region.set(root.attr("region").asVec4());
    } else {
        region.scale(glm::length(right), glm::length(up));
    }
    if (root.has("region-scale")) {
        region.scale(root.attr("region-scale").asVec2());
    }

    if (root.has("shading")) {
        shading = to_boolean(root.attr("shading"));
    }

    auto flip = root.attr("flip", "").getText();
    if (flip == "h") {
        std::swap(region.u1, region.u2);
        right *= -1;
        from -= right;
    } else if (flip == "v") {
        std::swap(region.v1, region.v2);
        up *= -1;
        from -= up;
    }
    std::string texture = root.attr("texture", "$0").getText();

    auto normal = glm::cross(glm::normalize(right), glm::normalize(up));
    builder.addRect(
        texture,
        shading,
        from + right * 0.5f + up * 0.5f,
        right * 0.5f,
        up * 0.5f,
        normal,
        region
    );
}

static void perform_triangle(const xmlelement& root, ModelBuilder& builder) {
    auto pointA = root.attr("a").asVec3();
    auto pointB = root.attr("b").asVec3();
    auto pointC = root.attr("c").asVec3();

    glm::vec2 uvs[3] {{0, 0}, {1, 0}, {1, 1}};

    bool shading = true;
    if (root.has("shading")) {
        shading = to_boolean(root.attr("shading"));
    }

    glm::vec3 ba = pointB - pointA;
    glm::vec3 ca = pointC - pointA;
    glm::vec3 normal = glm::normalize(glm::cross(ba, ca));
    
    if (root.has("uv")) {
        root.attr("uv").asNumbers(glm::value_ptr(uvs[0]), 6);
    } else {
        UVRegion region {};
        if (root.has("region")) {
            region.set(root.attr("region").asVec4());
        }
        if (root.has("region-scale")) {
            region.scale(root.attr("region-scale").asVec2());
        }
        for (auto& uv : uvs) {
            uv = region.apply(uv);
        }
    }
    
    std::string texture = root.attr("texture", "$0").getText();
    builder.addTriangle(texture, shading, pointA, pointB, pointC, normal, uvs[0], uvs[1], uvs[2]);
}

static void perform_box(const xmlelement& root, ModelBuilder& builder) {
    auto from = root.attr("from").asVec3();
    auto to = root.attr("to").asVec3();

    glm::vec3 origin = (from + to) * 0.5f;
    if (root.has("origin")) {
        origin = root.attr("origin").asVec3();
    }

    glm::mat4 tsf(1.0f);
    from -= origin;
    to -= origin;
    tsf = glm::translate(tsf, origin);

    if (root.has("rotate")) {
        auto text = root.attr("rotate").getText();
        if (std::count(text.begin(), text.end(), ',') == 3) {
            auto quat = root.attr("rotate").asVec4();
            tsf *= glm::mat4_cast(glm::quat(quat.w, quat.x, quat.y, quat.z));
        } else {
            auto rot = root.attr("rotate").asVec3();
            tsf = glm::rotate(tsf, glm::radians(rot.x), glm::vec3(1, 0, 0));
            tsf = glm::rotate(tsf, glm::radians(rot.y), glm::vec3(0, 1, 0));
            tsf = glm::rotate(tsf, glm::radians(rot.z), glm::vec3(0, 0, 1));
        }
    }

    UVRegion regions[6] {};
    regions[0].scale(to.x - from.x, to.y - from.y);
    regions[1].scale(from.x - to.x, to.y - from.y);
    regions[2].scale(to.x - from.x, to.z - from.z);
    regions[3].scale(from.x - to.x, to.z - from.z);
    regions[4].scale(to.z - from.z, to.y - from.y);
    regions[5].scale(from.z - to.z, to.y - from.y);

    auto center = (from + to) * 0.5f;
    auto halfsize = (to - from) * 0.5f;

    bool shading = true;
    std::string texfaces[6] {"$0","$1","$2","$3","$4","$5"};

    if (root.has("texture")) {
        auto texture = root.attr("texture").getText();
        for (int i = 0; i < 6; i++) {
            texfaces[i] = texture;
        }
    }

    if (root.has("shading")) {
        shading = to_boolean(root.attr("shading"));
    }

    for (const auto& elem : root.getElements()) {
        if (elem->getTag() == "part") {
            // todo: replace by expression parsing
            auto tags = util::split(elem->attr("tags").getText(), ',');
            for (auto& tag : tags) {
                util::trim(tag);
                const auto& found = side_indices.find(tag);
                if (found == side_indices.end()) {
                    continue;
                }
                int idx = found->second;
                if (elem->has("texture")) {
                    texfaces[idx] = elem->attr("texture").getText();
                }
                if (elem->has("region")) {
                    regions[idx].set(elem->attr("region").asVec4());
                }
                if (elem->has("region-scale")) {
                    regions[idx].scale(elem->attr("region-scale").asVec2());
                }
            }
        }
    }

    bool deleted[6] {};
    if (root.has("delete")) {
        // todo: replace with expression parsing
        auto names = util::split(root.attr("delete").getText(), ',');
        for (auto& name : names) {
            util::trim(name);
            const auto& found = side_indices.find(name);
            if (found != side_indices.end()) {
                deleted[found->second] = true;
            }
        }
    }

    builder.push(tsf);
    for (int i = 0; i < 6; i++) {
        if (deleted[i]) {
            continue;
        }
        bool enabled[6] {};
        enabled[i] = true;

        builder.addBox(texfaces[i], shading, center, halfsize, regions, enabled);
    }
    builder.pop();
}

static void perform_bone(const xmlelement& root, ModelBuilder& builder, Context& ctx) {
    std::string name = root.attr("name", "").getText();

    glm::vec3 movement {};
    glm::mat4 tsf(1.0f);
    if (root.has("move")) {
        movement = root.attr("move").asVec3();
    }
    if (root.has("rotate")) {
        auto text = root.attr("rotate").getText();
        if (std::count(text.begin(), text.end(), ',') == 3) {
            auto quat = root.attr("rotate").asVec4();
            tsf *= glm::mat4_cast(glm::quat(quat.w, quat.x, quat.y, quat.z));
        } else {
            auto rot = root.attr("rotate").asVec3();
            tsf = glm::rotate(tsf, glm::radians(rot.x), glm::vec3(1, 0, 0));
            tsf = glm::rotate(tsf, glm::radians(rot.y), glm::vec3(0, 1, 0));
            tsf = glm::rotate(tsf, glm::radians(rot.z), glm::vec3(0, 0, 1));
        }
    }
    if (root.has("scale")) {
        tsf = glm::scale(tsf, root.attr("scale").asVec3());
    }

    if (name.empty()) {
        tsf = glm::translate(tsf, movement);
        builder.push(std::move(tsf));
        for (const auto& elem : root.getElements()) {
            perform_element(*elem, builder, ctx);
        }
        builder.pop();
    } else {
        glm::vec3 origin = builder.getTransform() * glm::vec4(movement, 1.0f);
        size_t boneIndex = ctx.boneIndex++;
        tsf = builder.getTransform() * tsf;

        std::vector<std::unique_ptr<Bone>> bones;
        Context boneContext {ctx.vcmModel, bones, ctx.boneIndex};
        Model boneModel;
        ModelBuilder boneModelBuilder(boneModel);
        boneModelBuilder.push(std::move(tsf));
        for (const auto& elem : root.getElements()) {
            perform_element(*elem, boneModelBuilder, boneContext);
        }
        ctx.bones.push_back(std::make_unique<Bone>(
            boneIndex, name, name, std::move(bones), std::move(origin)
        ));
        ctx.vcmModel.parts[std::move(name)] = std::move(boneModel);
    }
}

static void perform_element(const xmlelement& root, ModelBuilder& builder, Context& ctx) {
    auto tag = root.getTag();

    if (tag == "rect") {
        perform_rect(root, builder);
    } else if (tag == "box") {
        perform_box(root, builder);
    } else if (tag == "tri") {
        perform_triangle(root, builder);
    } else if (tag == "bone") {
        perform_bone(root, builder, ctx);
    }
}

static VcmModel load_model(const xmlelement& root) {
    VcmModel vcmModel {};
    Model model;
    ModelBuilder builder(model);

    size_t boneIndex = 1;
    std::vector<std::unique_ptr<Bone>> bones;
    Context ctx { vcmModel, bones, boneIndex };

    for (const auto& elem : root.getElements()) {
        perform_element(*elem, builder, ctx);
    }

    vcmModel.parts["root"] = std::move(model);
    vcmModel.skeleton = SkeletonConfig(
        "",
        std::make_unique<Bone>(
            0, "root", "root", std::move(bones), glm::vec3(0.0f)
        ),
        boneIndex
    );
    return vcmModel;
}

VcmModel vcm::parse(
    std::string_view file, std::string_view src, bool usexml
) {
    try {
        auto doc =
            usexml ? xml::parse(file, src) : xml::parse_vcm(file, src, "model");
        const auto& root = *doc->getRoot();
        if (root.getTag() != "model") {
            throw std::runtime_error(
                "'model' tag expected as root, got '" + root.getTag() + "'"
            );
        }
        return load_model(root);
    } catch (const parsing_error& err) {
        throw std::runtime_error(err.errorLog());
    }
}
