#include "api_lua.hpp"

#include "../usertypes/lua_type_canvas.hpp"
#include "assets/Assets.hpp"
#include "assets/AssetsLoader.hpp"
#include "coders/obj.hpp"
#include "coders/png.hpp"
#include "coders/vcm.hpp"
#include "coders/vec3.hpp"
#include "debug/Logger.hpp"
#include "engine/Engine.hpp"
#include "graphics/commons/Model.hpp"
#include "graphics/core/Atlas.hpp"
#include "graphics/core/Texture.hpp"
#include "io/io.hpp"
#include "util/Buffer.hpp"

using namespace scripting;

static void load_texture(
    Assets& assets, const ubyte* bytes, size_t size, const std::string& destname
) {
    try {
        assets.store(png::load_texture(bytes, size), destname);
    } catch (const std::runtime_error& err) {
        debug::Logger logger("lua.assetslib");
        logger.error() << err.what();
    }
}

static void store_vcm(
    Assets& assets,
    const std::string& modelName,
    const std::string& skeletonName,
    vcm::VcmModel& vcmModel
) {
    auto skeleton = std::make_unique<rigging::SkeletonConfig>(
        std::move(*vcmModel.skeleton)
    );
    if (vcmModel.parts.size() > 1) {
        for (auto& [partName, model] : vcmModel.parts) {
            assets.store(
                std::make_unique<model::Model>(std::move(model)),
                modelName + "." + partName
            );
        }
        for (auto& bone : skeleton->getBones()) {
            bone->setModel(modelName + "." + bone->model.name);
        }
    } else {
        assets.store(
            std::make_unique<model::Model>(std::move(vcmModel.parts["root"])),
            modelName
        );
        skeleton->getRoot()->setModel(modelName);
    }
    assets.store(std::move(skeleton), skeletonName);
}

static int l_request_texture(lua::State* L) {
    std::string filename = lua::require_string(L, 1);
    std::string alias = lua::require_string(L, 2);
    auto& loader = engine->acquireBackgroundLoader();
    loader.add(AssetType::TEXTURE, filename, alias);
    return 0;
}

static int l_load_texture(lua::State* L) {
    auto& assets = engine->requireAssets();

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
        load_texture(
            assets, buffer.data(), buffer.size(), lua::require_string(L, 2)
        );
    } else {
        auto string = lua::bytearray_as_string(L, 1);
        load_texture(
            assets,
            reinterpret_cast<const ubyte*>(string.data()),
            string.size(),
            lua::require_string(L, 2)
        );
        lua::pop(L);
    }
    return 0;
}

static int l_parse_animation(lua::State* L) {
    auto format = lua::require_lstring(L, 1);
    auto string = lua::require_lstring(L, 2);
    std::string name = lua::require_string(L, 3);

    if (format != "vca") {
        throw std::runtime_error(
            "unknown format " + util::quote(std::string(format))
        );
    }
    scripting::load_vca_animation(name, &string, name);
    return 0;
}

static int l_parse_model(lua::State* L) {
    auto& assets = engine->requireAssets();

    auto format = lua::require_lstring(L, 1);
    std::string name = lua::require_string(L, 3);
    std::string skeletonName;
    if (lua::isstring(L, 4)) {
        skeletonName = lua::require_string(L, 4);
    }

    if (format == "obj") {
        auto string = lua::require_lstring(L, 2);
        assets.store(obj::parse(name, string), name);
        return 0;
    }

    if (format == "vec3") {
        auto content = lua::bytearray_as_string(L, 2);
        util::Buffer<ubyte> buffer(content.size());
        for (size_t i = 0; i < content.size(); i++) {
            buffer[i] = static_cast<ubyte>(content[i]);
        }
        auto vec3File = vec3::load(name, buffer);
        for (auto& [modelName, model] : vec3File.models) {
            auto fullName = name;
            if (name != modelName) {
                fullName += "." + modelName;
            }
            model.model.translate(model.origin);
            assets.store(
                std::make_unique<model::Model>(std::move(model.model)),
                fullName
            );
        }
        return 0;
    }

    if (format != "xml" && format != "vcm") {
        throw std::runtime_error(
            "unknown format " + util::quote(std::string(format))
        );
    }

    auto string = lua::require_lstring(L, 2);
    auto vcmModel = vcm::parse(name, string, format == "xml");

    if (skeletonName.empty()) {
        assets.store(
            std::make_unique<model::Model>(std::move(vcmModel.squash())), name
        );
        return 0;
    }

    if (!vcmModel.skeleton) {
        throw std::runtime_error(
            "model format " + util::quote(std::string(format)) +
            " does not define a skeleton"
        );
    }
    store_vcm(assets, name, skeletonName, vcmModel);
    return 0;
}

static int l_parse_skeleton(lua::State* L) {
    auto& assets = engine->requireAssets();

    auto format = lua::require_lstring(L, 1);
    auto string = lua::require_lstring(L, 2);
    std::string name = lua::require_string(L, 3);

    if (format == "json") {
        auto skeleton = rigging::SkeletonConfig::parse(string, name, name);
        assets.store(std::move(skeleton), name);
        return 0;
    }

    if (format != "xml" && format != "vcm") {
        throw std::runtime_error(
            "unknown format " + util::quote(std::string(format))
        );
    }

    auto vcmModel = vcm::parse(name, string, format == "xml");
    if (!vcmModel.skeleton) {
        throw std::runtime_error(
            "model format " + util::quote(std::string(format)) +
            " does not define a skeleton"
        );
    }

    assets.store(
        std::make_unique<rigging::SkeletonConfig>(std::move(*vcmModel.skeleton)),
        name
    );
    return 0;
}

static int l_to_canvas(lua::State* L) {
    auto& assets = engine->requireAssets();

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
    {"request_texture", lua::wrap<l_request_texture>},
    {"load_texture", lua::wrap<l_load_texture>},
    {"parse_animation", lua::wrap<l_parse_animation>},
    {"parse_model", lua::wrap<l_parse_model>},
    {"parse_skeleton", lua::wrap<l_parse_skeleton>},
    {"to_canvas", lua::wrap<l_to_canvas>},
    {nullptr, nullptr}
};
