#include "api_lua.hpp"

#include "logic/scripting/scripting_hud.hpp"
#include "graphics/render/WorldRenderer.hpp"
#include "graphics/render/BlockWrapsRenderer.hpp"

using namespace scripting;

static int l_wrap(lua::State* L) {
    auto position = lua::tovec3(L, 1);
    std::string texture = lua::require_string(L, 2);

    return lua::pushinteger(
        L, renderer->blockWraps->add(position, std::move(texture))
    );
}

static int l_unwrap(lua::State* L) {
    renderer->blockWraps->remove(lua::tointeger(L, 1));
    return 0;
}

static int l_set_pos(lua::State* L) {
    if (auto wrapper = renderer->blockWraps->get(lua::tointeger(L, 1))) {
        wrapper->position = lua::tovec3(L, 2);
    }
    return 0;
}

static int l_set_texture(lua::State* L) {
    if (auto wrapper = renderer->blockWraps->get(lua::tointeger(L, 1))) {
        wrapper->texture = lua::require_string(L, 2);
    }
    return 0;
}

const luaL_Reg blockwrapslib[] = {
    {"wrap", lua::wrap<l_wrap>},
    {"unwrap", lua::wrap<l_unwrap>},
    {"set_pos", lua::wrap<l_set_pos>},
    {"set_texture", lua::wrap<l_set_texture>},
    {nullptr, nullptr}
};
