#include "Project.hpp"

#include "data/dv_util.hpp"
#include "debug/Logger.hpp"
#include "engine/CoreParameters.hpp"
#include "io/io.hpp"
#include "io/path.hpp"
#include "logic/scripting/scripting.hpp"

static debug::Logger logger("project");

Project::~Project() = default;

dv::value Project::serialize() const {
    auto permissionsList = dv::list();
    for (const auto& perm : permissions.permissions) {
        permissionsList.add(perm);
    }
    return dv::object({
        {"name", name},
        {"title", title},
        {"base_packs", dv::to_value(basePacks)},
        {"permissions", std::move(permissionsList)}
    });
}

void Project::deserialize(const dv::value& src) {
    src.at("name").get(name);
    src.at("title").get(title);
    dv::get(src, "base_packs", basePacks);

    if (src.has("permissions")) {
        std::vector<std::string> perms;
        dv::get(src, "permissions", perms);
        permissions.permissions =
            std::set<std::string>(perms.begin(), perms.end());
    }
    logger.info() << "permissions: ";
    for (const auto& perm : permissions.permissions) {
        logger.info() << " - " << perm;
    }
}

bool Permissions::has(const std::string& name) const {
    return permissions.find(name) != permissions.end();
}
