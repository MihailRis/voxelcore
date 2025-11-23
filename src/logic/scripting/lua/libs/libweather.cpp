#include "libhud.hpp"

#include "world/Level.hpp"
#include "world/World.hpp"

using namespace scripting;

static Weather& require_weather() {
    if (level == nullptr) {
        throw std::runtime_error("world is not open");
    }
    return renderer->getWeather();
}

static int l_change(lua::State* L) {
    WeatherPreset preset {};
    preset.deserialize(lua::tovalue(L, 1));
    float time = lua::tonumber(L, 2);
    std::string name;
    if (lua::isstring(L, 3)) {
        name = lua::tostring(L, 3);
    }
    auto& weather = require_weather();
    weather.change(std::move(preset), time, std::move(name));
    return 0;
}

static int l_get_current(lua::State* L) {
    const auto& weather = require_weather();
    if (weather.t > 0.5f) {
        return lua::pushstring(L, weather.nameB);
    } else {
        return lua::pushstring(L, weather.nameA);
    }
}

static int l_get_fall_intensity(lua::State* L) {
    auto& weather = require_weather();
    const auto& a = weather.a;
    const auto& b = weather.b;
    float t = weather.t;
    return lua::pushnumber(L, 
        (a.fall.texture.empty() ? 0.0f : (1.0f - t)) +
        (b.fall.texture.empty() ? 0.0f : t)
    );
}

static int l_get_current_data(lua::State* L) {
    auto& weather = require_weather();
    if (weather.t > 0.5f) {
        return lua::pushvalue(L, weather.b.serialize());
    } else {
        return lua::pushvalue(L, weather.a.serialize());
    }
}

static int l_is_transition(lua::State* L) {
    return lua::pushboolean(L, require_weather().t < 1.0f);
}

const luaL_Reg weatherlib[] = {
    {"change", lua::wrap<l_change>},
    {"get_current", lua::wrap<l_get_current>},
    {"get_current_data", lua::wrap<l_get_current_data>},
    {"get_fall_intensity", lua::wrap<l_get_fall_intensity>},
    {"is_transition", lua::wrap<l_is_transition>},
    {nullptr, nullptr}
};
