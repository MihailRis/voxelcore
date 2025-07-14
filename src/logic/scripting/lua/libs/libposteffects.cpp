#include "libhud.hpp"

#include "assets/Assets.hpp"
#include "content/Content.hpp"
#include "graphics/core/PostEffect.hpp"
#include "graphics/core/PostProcessing.hpp"

using namespace scripting;

static int l_index(lua::State* L) {
    auto name = lua::require_string(L, 1);
    auto& indices = content->getIndices(ResourceType::POST_EFFECT_SLOT);
    return lua::pushinteger(L, indices.indexOf(name));
}

static int l_set_effect(lua::State* L) {
    size_t index = static_cast<size_t>(lua::tointeger(L, 1));
    auto name = lua::require_string(L, 2);
    auto& assets = *engine->getAssets();
    auto effect = assets.getShared<PostEffect>(name);
    if (effect == nullptr) {
        throw std::runtime_error(std::string("post-effect '") + name + "' not found");
    }
    post_processing->setEffect(index, std::move(effect));
    return 0;
}

static int l_get_intensity(lua::State* L) {
    size_t index = static_cast<size_t>(lua::tointeger(L, 1));
    auto effect = post_processing->getEffect(index);
    return lua::pushnumber(L, effect ? effect->getIntensity() : 0.0f);
}

static int l_set_intensity(lua::State* L) {
    size_t index = static_cast<size_t>(lua::tointeger(L, 1));
    float value = lua::tonumber(L, 2);
    post_processing->getEffect(index)->setIntensity(value);
    return 0;
}

static int l_is_active(lua::State* L) {
    size_t index = static_cast<size_t>(lua::tointeger(L, 1));
    auto effect = post_processing->getEffect(index);
    return lua::pushboolean(L, effect ? effect->isActive() : false);
}

static int l_set_params(lua::State* L) {
    size_t index = static_cast<size_t>(lua::tointeger(L, 1));
    auto table = lua::tovalue(L, 2);
    if (!table.isObject()) {
        throw std::runtime_error("params table expected");
    }
    auto effect = post_processing->getEffect(index);
    if (effect == nullptr) {
        return 0;
    }
    for (const auto& [key, value] : table.asObject()) {
        effect->setParam(key, value);
    }
    return 0;
}

static int l_set_array(lua::State* L) {
    size_t index = static_cast<size_t>(lua::tointeger(L, 1));
    auto key = lua::require_string(L, 2);
    auto data = lua::require_lstring(L, 3);
    auto effect = post_processing->getEffect(index);
    if (effect == nullptr) {
        return 0;
    }
    std::vector<ubyte> buffer(
        reinterpret_cast<const ubyte*>(data.data()),
        reinterpret_cast<const ubyte*>(data.data() + data.size())
    );
    effect->setArray(key, std::move(buffer));
    return 0;
}

const luaL_Reg posteffectslib[] = {
    {"index", lua::wrap<l_index>},
    {"set_effect", lua::wrap<l_set_effect>},
    {"get_intensity", lua::wrap<l_get_intensity>},
    {"set_intensity", lua::wrap<l_set_intensity>},
    {"is_active", lua::wrap<l_is_active>},
    {"set_params", lua::wrap<l_set_params>},
    {"set_array", lua::wrap<l_set_array>},
    {NULL, NULL}
};
