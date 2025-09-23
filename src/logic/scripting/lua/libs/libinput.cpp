#include "engine/Engine.hpp"
#include "content/ContentControl.hpp"
#include "frontend/hud.hpp"
#include "frontend/screens/Screen.hpp"
#include "graphics/ui/GUI.hpp"
#include "graphics/ui/elements/Container.hpp"
#include "io/io.hpp"
#include "libgui.hpp"
#include "util/stringutil.hpp"
#include "util/observer_handler.hpp"
#include "window/input.hpp"
#include "coders/toml.hpp"

namespace scripting {
    extern Hud* hud;
}
using namespace scripting;

static int l_keycode(lua::State* L) {
    auto name = lua::require_string(L, 1);
    return lua::pushinteger(
        L, static_cast<int>(input_util::keycode_from(name))
    );
}

static int l_mousecode(lua::State* L) {
    auto name = lua::require_string(L, 1);
    return lua::pushinteger(
        L, static_cast<int>(input_util::mousecode_from(name))
    );
}

static int l_add_callback(lua::State* L) {
    if (engine->isHeadless())
        return 0;
    std::string bindname = lua::require_string(L, 1);
    size_t pos = bindname.find(':');

    lua::pushvalue(L, 2);
    auto actual_callback = lua::create_simple_handler(L);
    ObserverHandler handler;

    auto& gui = engine->getGUI();
    auto& input = engine->getInput();

    if (pos != std::string::npos) {
        std::string prefix = bindname.substr(0, pos);
        if (prefix == "key") {
            auto key = input_util::keycode_from(bindname.substr(pos + 1));
            handler = input.addKeyCallback(key, actual_callback);
        }
    }
    auto callback = [&gui, actual_callback]() -> bool {
        if (!gui.isFocusCaught()) {
            return actual_callback();
        }
        return false;
    };
    if (handler == nullptr) {
        auto& bind = input.getBindings().require(bindname);
        handler = bind.onactived.add(callback);
    }

    if (hud) {
        hud->keepAlive(std::move(handler));
        return 0;
    } else if (lua::gettop(L) >= 3) {
        auto node = get_document_node(L, 3);
        if (auto container =
                std::dynamic_pointer_cast<gui::Container>(node.node)) {
            container->keepAlive(std::move(handler));
            return 0;
        }
        throw std::runtime_error("owner expected to be a container");
    }
    throw std::runtime_error("on_hud_open is not called yet");
}

static int l_get_mouse_pos(lua::State* L) {
    if (engine->isHeadless())
        return 0;
    return lua::pushvec2(L, engine->getInput().getCursor().pos);
}

static int l_get_bindings(lua::State* L) {
    if (engine->isHeadless())
        return 0;
    const auto& bindings = engine->getInput().getBindings().getAll();
    lua::createtable(L, bindings.size(), 0);

    int i = 0;
    for (auto& entry : bindings) {
        lua::pushstring(L, entry.first);
        lua::rawseti(L, i + 1);
        i++;
    }
    return 1;
}

static int l_get_binding_text(lua::State* L) {
    if (engine->isHeadless())
        return 0;
    auto bindname = lua::require_string(L, 1);
    const auto& bind = engine->getInput().getBindings().require(bindname);
    return lua::pushstring(L, bind.text());
}

static int l_is_active(lua::State* L) {
    if (engine->isHeadless())
        return 0;
    auto bindname = lua::require_string(L, 1);
    auto& bind = engine->getInput().getBindings().require(bindname);
    return lua::pushboolean(L, bind.active());
}

static int l_is_pressed(lua::State* L) {
    if (engine->isHeadless())
        return 0;
    std::string code = lua::require_string(L, 1);
    size_t sep = code.find(':');
    if (sep == std::string::npos) {
        throw std::runtime_error("expected 'input_type:key' format");
    }
    auto prefix = code.substr(0, sep);
    auto name = code.substr(sep + 1);
    if (prefix == "key") {
        return lua::pushboolean(
            L, engine->getInput().pressed(input_util::keycode_from(name))
        );
    } else if (prefix == "mouse") {
        return lua::pushboolean(
            L, engine->getInput().clicked(input_util::mousecode_from(name))
        );
    } else {
        throw std::runtime_error("unknown input type " + util::quote(code));
    }
}

static void reset_pack_bindings(const io::path& packFolder) {
    auto configFolder = packFolder / "config";
    auto bindsFile = configFolder / "bindings.toml";
    if (io::is_regular_file(bindsFile)) {
        engine->getInput().getBindings().read(
            toml::parse(bindsFile.string(), io::read_string(bindsFile)),
            BindType::REBIND
        );
    }
}

static int l_reset_bindings(lua::State*) {
    if (engine->isHeadless())
        return 0;
    reset_pack_bindings("res:");
    for (const auto& pack : content_control->getContentPacks()) {
        reset_pack_bindings(pack.folder);
    }
    return 0;
}

static int l_set_enabled(lua::State* L) {
    if (engine->isHeadless())
        return 0;
    std::string bindname = lua::require_string(L, 1);
    bool enabled = lua::toboolean(L, 2);
    engine->getInput().getBindings().require(bindname).enabled = enabled;
    return 0;
}

const luaL_Reg inputlib[] = {
    {"keycode", lua::wrap<l_keycode>},
    {"mousecode", lua::wrap<l_mousecode>},
    {"add_callback", lua::wrap<l_add_callback>},
    {"get_mouse_pos", lua::wrap<l_get_mouse_pos>},
    {"get_bindings", lua::wrap<l_get_bindings>},
    {"get_binding_text", lua::wrap<l_get_binding_text>},
    {"is_active", lua::wrap<l_is_active>},
    {"is_pressed", lua::wrap<l_is_pressed>},
    {"reset_bindings", lua::wrap<l_reset_bindings>},
    {"set_enabled", lua::wrap<l_set_enabled>},
    {NULL, NULL}
};
