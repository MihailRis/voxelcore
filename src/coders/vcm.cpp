#include "vcm.hpp"

#include <iostream>

#include "xml.hpp"
#include "util/stringutil.hpp"
#include "graphics/commons/Model.hpp"
#include "io/io.hpp"

using namespace vcm;
using namespace xml;

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

static void perform_rect(const xmlelement& root, model::Model& model) {
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
    auto& mesh = model.addMesh(texture, shading);

    auto normal = glm::cross(glm::normalize(right), glm::normalize(up));
    mesh.addRect(
        from + right * 0.5f + up * 0.5f,
        right * 0.5f,
        up * 0.5f,
        normal,
        region
    );
}

static void perform_box(const xmlelement& root, model::Model& model) {
    auto from = root.attr("from").asVec3();
    auto to = root.attr("to").asVec3();

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
        // todo: replace by expression parsing
        auto names = util::split(root.attr("delete").getText(), ',');
        for (auto& name : names) {
            util::trim(name);
            const auto& found = side_indices.find(name);
            if (found != side_indices.end()) {
                deleted[found->second] = true;
            }
        }
    }

    for (int i = 0; i < 6; i++) {
        if (deleted[i]) {
            continue;
        }
        bool enabled[6] {};
        enabled[i] = true;
        auto& mesh = model.addMesh(texfaces[i], shading);
        mesh.addBox(center, halfsize, regions, enabled);
    }
}

static std::unique_ptr<model::Model> load_model(const xmlelement& root) {
    model::Model model;

    for (const auto& elem : root.getElements()) {
        auto tag = elem->getTag();

        if (tag == "rect") {
            perform_rect(*elem, model);
        } else if (tag == "box") {
            perform_box(*elem, model);
        }
    }

    return std::make_unique<model::Model>(std::move(model));
}

std::unique_ptr<model::Model> vcm::parse(
    std::string_view file, std::string_view src
) {
    try {
        auto doc = io::path(std::string(file)).extension() == ".xml" 
                    ? xml::parse(file, src) : xml::parse_vcm(file, src, "model");
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
