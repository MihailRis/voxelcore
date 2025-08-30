#include "scripting_hud.hpp"

#include "debug/Logger.hpp"
#include "engine/Engine.hpp"
#include "io/io.hpp"
#include "assets/Assets.hpp"
#include "content/ContentControl.hpp"
#include "frontend/hud.hpp"
#include "frontend/UiDocument.hpp"
#include "graphics/render/WorldRenderer.hpp"
#include "objects/Player.hpp"
#include "lua/libs/api_lua.hpp"
#include "lua/lua_engine.hpp"
#include "scripting.hpp"

using namespace scripting;

static debug::Logger logger("scripting-hud");

Hud* scripting::hud = nullptr;
WorldRenderer* scripting::renderer = nullptr;
PostProcessing* scripting::post_processing = nullptr;

static void load_script(const std::string& name) {
    auto file = io::path("res:scripts") / name;
    std::string src = io::read_string(file);
    logger.info() << "loading script " << file.string();

    lua::execute(lua::get_main_state(), 0, src, file.string());
}

void scripting::on_frontend_init(
    Hud* hud, WorldRenderer* renderer, PostProcessing* postProcessing
) {
    scripting::hud = hud;
    scripting::renderer = renderer;
    scripting::post_processing = postProcessing;

    auto L = lua::get_main_state();

    lua::openlib(L, "hud", hudlib);
    lua::openlib(L, "gfx", "blockwraps", blockwrapslib);
    lua::openlib(L, "gfx", "particles", particleslib);
    lua::openlib(L, "gfx", "weather", weatherlib);
    lua::openlib(L, "gfx", "text3d", text3dlib);
    lua::openlib(L, "gfx", "posteffects", posteffectslib);

    load_script("hud_classes.lua");

    if (lua::getglobal(L, "__vc_on_hud_open")) {
        lua::call_nothrow(L, 0, 0);
    }

    for (auto& pack : content_control->getAllContentPacks()) {
        lua::emit_event(
            lua::get_main_state(),
            pack.id + ":.hudopen",
            [&](lua::State* L) {
                return lua::pushinteger(L, hud->getPlayer()->getId());
            }
        );
    }
}

void scripting::on_frontend_render() {
    for (auto& pack : content_control->getAllContentPacks()) {
        lua::emit_event(
            lua::get_main_state(),
            pack.id + ":.hudrender",
            [](lua::State* L) { return 0; }
        );
    }
}

void scripting::on_frontend_close() {
    auto L = lua::get_main_state();
    for (auto& pack : content_control->getAllContentPacks()) {
        lua::emit_event(
            L,
            pack.id + ":.hudclose",
            [&](lua::State* L) {
                return lua::pushinteger(L, hud->getPlayer()->getId());
            }
        );
    }
    lua::pushnil(L);
    lua::setglobal(L, "hud");
    lua::pushnil(L);
    lua::setglobal(L, "gfx");

    scripting::renderer = nullptr;
    scripting::hud = nullptr;
    scripting::post_processing = nullptr;
}

void scripting::load_hud_script(
    const scriptenv& senv,
    const std::string& packid,
    const io::path& file,
    const std::string& fileName
) {
    int env = *senv;
    std::string src = io::read_string(file);
    logger.info() << "loading script " << file.string();

    lua::execute(lua::get_main_state(), env, src, fileName);

    register_event(env, "init", packid + ":.init");
    register_event(env, "on_hud_open", packid + ":.hudopen");
    register_event(env, "on_hud_render", packid + ":.hudrender");
    register_event(env, "on_hud_close", packid + ":.hudclose");
}

gui::PageLoaderFunc scripting::create_page_loader() {
    auto L = lua::get_main_state();
    if (lua::getglobal(L, "__vc_page_loader")) {
        auto func = lua::create_lambda(L);
        return [func](const std::string& name) -> std::shared_ptr<gui::UINode> {
            auto docname = func({name}).asString();
            return Engine::getInstance().getAssets()->require<UiDocument>(docname).getRoot();
        };
    }
    return nullptr;
}
