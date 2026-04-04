#include "api_lua.hpp"

#include "logic/scripting/scripting_hud.hpp"
#include "graphics/render/WorldRenderer.hpp"
#include "graphics/render/BlockWrapsRenderer.hpp"

using namespace scripting;

static int l_wrap(lua::State* L) {
    auto position = lua::tovec3(L, 1);
    std::string texture = lua::require_string(L, 2);
    float emission = lua::isnumber(L, 3) ? lua::tonumber(L, 3) : 1.0f;
    glm::vec3 tint = lua::istable(L, 4) ? lua::tovec3(L, 4) : glm::vec3(1.0f);

    return lua::pushinteger(
        L,
        renderer->blockWraps->add(
            position, std::move(texture), std::move(tint), emission
        )
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
        const char* newTexture = lua::require_string(L, 2);

        for (int i = 0; i < wrapper->textureFaces.size(); i++) {
            if (wrapper->textureFaces[i] != newTexture) {
                wrapper->textureFaces[i] = newTexture;
                wrapper->dirtySides = 0xFF;
            }
        }
    }
    return 0;
}

static int l_set_faces(lua::State* L) {
    if (auto wrapper = renderer->blockWraps->get(lua::tointeger(L, 1))) {
        for (int i = 0; i < wrapper->textureFaces.size(); i++) {
            if (lua::isnil(L, 2 + i)) {
                if (wrapper->cullingBits & (1 << i)) {
                    wrapper->cullingBits &= ~(1 << i);
                    wrapper->textureFaces[i] = "";
                    wrapper->dirtySides |= (1 << i);
                }
            } else {
                auto texture = lua::require_string(L, 2 + i);
                if ((wrapper->cullingBits & (1 << i)) == 0x0
                 || wrapper->textureFaces[i] != texture) {
                    wrapper->cullingBits |= (1 << i);
                    wrapper->textureFaces[i] = texture;
                }
            }
        }
    }
    return 0;
}

static int l_set_tints(lua::State* L) {
    if (auto wrapper = renderer->blockWraps->get(lua::tointeger(L, 1))) {
        for (int i = 0; i < wrapper->textureFaces.size(); i++) {
            if (lua::isnil(L, 2 + i)) {
                wrapper->tints[i] = glm::vec3(1.0f);
            } else {
                wrapper->tints[i] = lua::tovec3(L, 2 + i);
            }
        }
    }
    return 0;
}

const luaL_Reg blockwrapslib[] = {
    {"wrap", lua::wrap<l_wrap>},
    {"unwrap", lua::wrap<l_unwrap>},
    {"set_pos", lua::wrap<l_set_pos>},
    {"set_texture", lua::wrap<l_set_texture>},
    {"set_faces", lua::wrap<l_set_faces>},
    {"set_tints", lua::wrap<l_set_tints>},
    {nullptr, nullptr}
};
