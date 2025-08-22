#include "api_lua.hpp"

#include "io/io.hpp"
#include "io/util.hpp"
#include "coders/binary_json.hpp"
#include "world/Level.hpp"
#include "world/generator/VoxelFragment.hpp"
#include "content/ContentLoader.hpp"
#include "content/Content.hpp"
#include "content/ContentControl.hpp"
#include "engine/Engine.hpp"
#include "../lua_custom_types.hpp"

using namespace scripting;

static int l_save_fragment(lua::State* L) {
    auto fragment = lua::touserdata<lua::LuaVoxelFragment>(L, 1);
    auto file = lua::require_string(L, 2);
    auto map = fragment->getFragment(0)->serialize();
    auto bytes = json::to_binary(map, true);
    io::write_bytes(file, bytes.data(), bytes.size());
    return 0;
}

static int l_create_fragment(lua::State* L) {
    auto pointA = lua::tovec<3>(L, 1);
    auto pointB = lua::tovec<3>(L, 2);
    bool crop = lua::toboolean(L, 3);
    bool saveEntities = lua::toboolean(L, 4);

    auto fragment =
        VoxelFragment::create(*level, pointA, pointB, crop, saveEntities);
    std::array<std::shared_ptr<VoxelFragment>, 4> fragmentVariants {
        std::move(fragment)
    };
    for (size_t i = 1; i < 4; i++) {
        fragmentVariants[i] = fragmentVariants[i - 1]->rotated(*content);
    }
    return lua::newuserdata<lua::LuaVoxelFragment>(
        L, std::move(fragmentVariants)
    );
}

static int l_load_fragment(lua::State* L) {
    io::path path = lua::require_string(L, 1);
    if (!io::exists(path)) {
        throw std::runtime_error("file "+path.string()+" does not exist");
    }
    auto map = io::read_binary_json(path);

    auto fragment = std::make_shared<VoxelFragment>();
    fragment->deserialize(map);
    fragment->prepare(*content);
    std::array<std::shared_ptr<VoxelFragment>, 4> fragmentVariants {
        std::move(fragment)
    };
    for (size_t i = 1; i < 4; i++) {
        fragmentVariants[i] = fragmentVariants[i - 1]->rotated(*content);
    }
    return lua::newuserdata<lua::LuaVoxelFragment>(
        L, std::move(fragmentVariants)
    );
}

/// @brief Get a list of all world generators
/// @return A table with the IDs of all world generators
static int l_get_generators(lua::State* L) {
    auto packs = content_control->getAllContentPacks();

    lua::createtable(L, 0, 0);

    for (const auto& pack : packs) {
        auto pairs = ContentLoader::scanContent(pack, ContentType::GENERATOR);
        for (const auto& [name, caption] : pairs) {
            lua::pushstring(L, caption);
            lua::setfield(L, name);
        }
    }
    return 1;
}

/// @brief Get the default world generator
/// @return The ID of the default world generator
static int l_get_default_generator(lua::State* L) {
    // content is not initialized yet
    auto combined = engine->getResPaths().readCombinedObject(
        EnginePaths::CONFIG_DEFAULTS.string()
    );
    return lua::pushstring(L, combined["generator"].asString());
}

const luaL_Reg generationlib[] = {
    {"create_fragment", lua::wrap<l_create_fragment>},
    {"save_fragment", lua::wrap<l_save_fragment>},
    {"load_fragment", lua::wrap<l_load_fragment>},
    {"get_generators", lua::wrap<l_get_generators>},
    {"get_default_generator", lua::wrap<l_get_default_generator>},
    {NULL, NULL}
};
