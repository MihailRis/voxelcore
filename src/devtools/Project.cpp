#include "Project.hpp"

#include "data/dv_util.hpp"
#include "logic/scripting/scripting.hpp"

Project::~Project() = default;

dv::value Project::serialize() const {
    return dv::object({
        {"name", name},
        {"title", title},
        {"base_packs", dv::to_value(basePacks)},
    });
}

void Project::deserialize(const dv::value& src) {
    src.at("name").get(name);
    src.at("title").get(title);
    dv::get(src, "base_packs", basePacks);
}
