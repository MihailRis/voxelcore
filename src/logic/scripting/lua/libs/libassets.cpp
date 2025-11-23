#include "api_lua.hpp"

#include "assets/Assets.hpp"
#include "coders/png.hpp"
#include "coders/vcm.hpp"
#include "debug/Logger.hpp"
#include "engine/Engine.hpp"
#include "graphics/commons/Model.hpp"
#include "graphics/core/Texture.hpp"
#include "graphics/core/Atlas.hpp"
#include "util/Buffer.hpp"
#include "../usertypes/lua_type_canvas.hpp"

using namespace scripting;

static void load_texture(
    const ubyte* bytes, size_t size, const std::string& destname
) {
    try {
        engine->getAssets()->store(png::load_texture(bytes, size), destname);
    } catch (const std::runtime_error& err) {
        debug::Logger logger("lua.assetslib");
        logger.error() << err.what();
    }
}

static int l_load_texture(lua::State* L) {
    if (lua::isstring(L, 3) && lua::require_lstring(L, 3) != "png") {
        throw std::runtime_error("unsupportd image format");
    }
    if (lua::istable(L, 1)) {
        lua::pushvalue(L, 1);
        size_t size = lua::objlen(L, 1);
        util::Buffer<ubyte> buffer(size);
        for (size_t i = 0; i < size; i++) {
            lua::rawgeti(L, i + 1);
            buffer[i] = lua::tointeger(L, -1);
            lua::pop(L);
        }
        lua::pop(L);
        load_texture(buffer.data(), buffer.size(), lua::require_string(L, 2));
    } else {
        auto string = lua::bytearray_as_string(L, 1);
        load_texture(
            reinterpret_cast<const ubyte*>(string.data()),
            string.size(),
            lua::require_string(L, 2)
        );
        lua::pop(L);
    }
    return 0;
}

static int l_parse_model(lua::State* L) {
    auto format = lua::require_lstring(L, 1);
    auto string = lua::require_lstring(L, 2);
    auto name = lua::require_string(L, 3);
    
    if (format == "xml" || format == "vcm") {
        engine->getAssets()->store(
            vcm::parse(name, string, format == "xml"), name
        );
    } else {
        throw std::runtime_error("unknown format " + util::quote(std::string(format)));
    }
    return 0;
}

static int l_to_canvas(lua::State* L) {
    auto& assets = *engine->getAssets();

    auto alias = lua::require_lstring(L, 1);
    size_t sep = alias.rfind(':');
    if (sep == std::string::npos) {
        auto texture = assets.getShared<Texture>(std::string(alias));
        if (texture != nullptr) {
            auto image = texture->readData();
            return lua::newuserdata<lua::LuaCanvas>(
                L, texture, std::move(image)
            );
        }
        return 0;
    }
    auto atlasName = alias.substr(0, sep);
    
    if (auto atlas = assets.get<Atlas>(std::string(atlasName))) {
        auto textureName = std::string(alias.substr(sep + 1));
        auto image = atlas->shareImageData();
        auto texture = atlas->shareTexture();

        if (auto region = atlas->getIf(textureName)) {
            UVRegion uvRegion = *region;
            int atlasWidth = static_cast<int>(image->getWidth());
            int atlasHeight = static_cast<int>(image->getHeight());
            int x = static_cast<int>(uvRegion.u1 * atlasWidth);
            int y = static_cast<int>(uvRegion.v1 * atlasHeight);
            int w = static_cast<int>(uvRegion.getWidth() * atlasWidth);
            int h = static_cast<int>(uvRegion.getHeight() * atlasHeight);
            return lua::newuserdata<lua::LuaCanvas>(
                L, std::move(texture), image->cropped(x, y, w, h), uvRegion
            );
        }
    }
    return 0;
}

const luaL_Reg assetslib[] = {
    {"load_texture", lua::wrap<l_load_texture>},
    {"parse_model", lua::wrap<l_parse_model>},
    {"to_canvas", lua::wrap<l_to_canvas>},
    {nullptr, nullptr}
};
