#include "../lua_custom_types.hpp"

#include "../lua_util.hpp"

#include "world/generator/VoxelFragment.hpp"
#include "util/stringutil.hpp"
#include "world/Level.hpp"

using namespace lua;

LuaVoxelFragment::LuaVoxelFragment(
    std::array<std::shared_ptr<VoxelFragment>, 4> fragmentVariants
)
    : fragmentVariants(std::move(fragmentVariants)) {
}

LuaVoxelFragment::~LuaVoxelFragment() {
}

static int l_crop(lua::State* L) {
    if (auto fragment = touserdata<LuaVoxelFragment>(L, 1)) {
        for (size_t i = 0; i < 4; i++) {
            fragment->getFragment(i)->crop();
        }
    }
    return 0;
}

static int l_place(lua::State* L) {
    if (auto fragment = touserdata<LuaVoxelFragment>(L, 1)) {
        auto offset = tovec3(L, 2);
        int rotation = tointeger(L, 3);
        fragment->getFragment(rotation)->place(
            *scripting::level->chunks, offset 
        );
    }
    return 0;
}

static std::unordered_map<std::string, lua_CFunction> methods {
    {"crop", lua::wrap<l_crop>},
    {"place", lua::wrap<l_place>},
};

static int l_meta_tostring(lua::State* L) {
    return pushstring(L, "VoxelFragment(0x" + util::tohex(
        reinterpret_cast<uint64_t>(topointer(L, 1)))+")");
}

static int l_meta_index(lua::State* L) {
    auto fragment = touserdata<LuaVoxelFragment>(L, 1);
    if (fragment == nullptr) {
        return 0;
    }
    if (isstring(L, 2)) {
        auto fieldname = tostring(L, 2);
        if (!std::strcmp(fieldname, "size")) {
            return pushivec(L, fragment->getFragment(0)->getSize());
        } else {
            auto found = methods.find(tostring(L, 2));
            if (found != methods.end()) {
                return pushcfunction(L, found->second);
            }
        }
    }
    return 0;
}

int LuaVoxelFragment::createMetatable(lua::State* L) {
    createtable(L, 0, 2);
    pushcfunction(L, lua::wrap<l_meta_tostring>);
    setfield(L, "__tostring");
    pushcfunction(L, lua::wrap<l_meta_index>);
    setfield(L, "__index");
    return 1;
}
