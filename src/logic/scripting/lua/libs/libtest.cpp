#include "api_lua.hpp"
#include "engine/Engine.hpp"
#include "window/input.hpp"

#include <iostream>

using namespace scripting;

static int l_set_button_pressed(lua::State* L) {
    int button = lua::touinteger(L, 1);
    int x = lua::tointeger(L, 2);
    int y = lua::tointeger(L, 3);
    bool pressed = lua::toboolean(L, 4);
    
    auto& input = engine->getInput();
    input.simulateCursorPos(x, y);
    input.simulateClick(button, pressed);
    return 0;
}

static int l_set_key_pressed(lua::State* L) {
    auto key = lua::require_string(L, 1);
    bool pressed = lua::toboolean(L, 2);
    
    auto& input = engine->getInput();
    input.simulateKey(input_util::keycode_from(key), pressed);
    return 0;
}

static int l_enter_text(lua::State* L) {
    auto string = lua::require_wstring(L, 1);
    
    auto& input = engine->getInput();
    for (auto chr : string) {
        input.simulateCodepoint(chr);
    }
    return 0;
}

const luaL_Reg testlib[] = {
    {"set_button_pressed", lua::wrap<l_set_button_pressed>},
    {"set_key_pressed", lua::wrap<l_set_key_pressed>},
    {"enter_text", lua::wrap<l_enter_text>},
    {nullptr, nullptr}
};
