#define VC_ENABLE_REFLECTION
#include "lua_type_canvas.hpp"

#include "graphics/core/ImageData.hpp"
#include "graphics/core/Texture.hpp"
#include "logic/scripting/lua/lua_util.hpp"
#include "coders/imageio.hpp"
#include "engine/Engine.hpp"
#include "assets/Assets.hpp"

#include <unordered_map>

using namespace lua;

LuaCanvas::LuaCanvas(
    std::shared_ptr<Texture> texture,
    std::shared_ptr<ImageData> data,
    UVRegion region
)
    : texture(std::move(texture)),
      data(std::move(data)),
      region(std::move(region)) {
}

void LuaCanvas::update(int extrusion) {
    if (!hasTexture()) {
        return;
    }
    if (region.isFull()) {
        texture->reload(*data);
    } else {
        uint texWidth = texture->getWidth();
        uint texHeight = texture->getHeight();
        uint imgWidth = data->getWidth();
        uint imgHeight = data->getHeight();

        uint x = static_cast<uint>(region.u1 * texWidth);
        uint y = static_cast<uint>(region.v1 * texHeight);
        uint w = static_cast<uint>((region.u2 - region.u1) * texWidth);
        uint h = static_cast<uint>((region.v2 - region.v1) * texHeight);

        w = std::min<uint>(w, imgWidth);
        h = std::min<uint>(h, imgHeight);

        if (extrusion > 0) {
            auto extruded = std::make_unique<ImageData>(
                data->getFormat(),
                w + extrusion * 2,
                h + extrusion * 2
            );
            extruded->blit(*data, extrusion, extrusion);
            for (uint j = 0; j < extrusion; j++) {
                extruded->extrude(extrusion - j, extrusion - j, w + j*2, h + j*2);
            }
            texture->reloadPartial(
                *extruded,
                x - extrusion,
                y - extrusion,
                w + extrusion * 2,
                h + extrusion * 2
            );
        } else {
            texture->reloadPartial(*data, x, y, w, h);
        }
    }
}

void LuaCanvas::createTexture() {
    texture = Texture::from(data.get());
    texture->setMipMapping(false, true);
}

void LuaCanvas::unbindTexture() {
    texture.reset();
}

union RGBA {
    struct {
        uint8_t r, g, b, a;
    };
    uint8_t arr[4];
    uint32_t rgba;
};

static RGBA* get_at(const ImageData& data, uint index) {
    if (index >= data.getWidth() * data.getHeight()) {
        return nullptr;
    }
    return reinterpret_cast<RGBA*>(data.getData() + index * sizeof(RGBA));
}

static RGBA* get_at(const ImageData& data, uint x, uint y) {
    return get_at(data, y * data.getWidth() + x);
}

static RGBA* get_at(State* L, uint x, uint y) {
    if (auto canvas = touserdata<LuaCanvas>(L, 1)) {
        return get_at(canvas->getData(), x, y);
    }
    return nullptr;
}

static int l_at(State* L) {
    auto x = static_cast<uint>(tointeger(L, 2));
    auto y = static_cast<uint>(tointeger(L, 3));

    if (auto pixel = get_at(L, x, y)) {
        return pushinteger(L, pixel->rgba);
    }
    return 0;
}

static int l_unbind_texture(State* L) {
    if (auto canvas = touserdata<LuaCanvas>(L, 1)) {
        canvas->unbindTexture();
    }
    return 0;
}

static RGBA get_rgba(State* L, int first) {
    RGBA rgba {};
    rgba.a = 255;
    switch (gettop(L) - first) {
        case 0:
            rgba.rgba = static_cast<uint>(tointeger(L, first));
            break;
        case 3:
            if (lua::isnumber(L, first + 3)) {
                rgba.a = static_cast<ubyte>(tointeger(L, first + 3));
            }
            [[fallthrough]];
        case 2:
            rgba.r = static_cast<ubyte>(tointeger(L, first));
            rgba.g = static_cast<ubyte>(tointeger(L, first + 1));
            rgba.b = static_cast<ubyte>(tointeger(L, first + 2));
            break;
    }
    return rgba;
}

static int l_set(State* L) {
    auto x = static_cast<uint>(tointeger(L, 2));
    auto y = static_cast<uint>(tointeger(L, 3));

    if (auto pixel = get_at(L, x, y)) {
        *pixel = get_rgba(L, 4);
    }
    return 0;
}

static LuaCanvas& require_canvas(State* L, int idx) {
    if (const auto canvas = touserdata<LuaCanvas>(L, idx)) {
        return *canvas;
    }
    throw std::runtime_error(
        "canvas expected as argument #" + std::to_string(idx)
    );
}

static int l_clear(State* L) {
    auto& canvas = require_canvas(L, 1);
    auto& image = canvas.getData();
    ubyte* data = image.getData();
    RGBA rgba {};
    if (gettop(L) == 1) {
        std::fill(data, data + image.getDataSize(), 0);
        return 0;
    }
    rgba = get_rgba(L, 2);
    size_t pixels = image.getWidth() * image.getHeight();
    const size_t channels = 4;
    for (size_t i = 0; i < pixels * channels; i++) {
        data[i] = rgba.arr[i % channels];
    }
    return 0;
}

static int l_line(State* L) {
    int x1 = tointeger(L, 2);
    int y1 = tointeger(L, 3);

    int x2 = tointeger(L, 4);
    int y2 = tointeger(L, 5);

    RGBA rgba = get_rgba(L, 6);
    if (auto canvas = touserdata<LuaCanvas>(L, 1)) {
        auto& image = canvas->getData();
        image.drawLine(
            x1, y1, x2, y2, glm::ivec4 {rgba.r, rgba.g, rgba.b, rgba.a}
        );
    }
    return 0;
}

static int l_blit(State* L) {
    auto& dst = require_canvas(L, 1);
    auto& src = require_canvas(L, 2);
    int dst_x = tointeger(L, 3);
    int dst_y = tointeger(L, 4);
    dst.getData().blit(src.getData(), dst_x, dst_y);
    return 0;
}

static int l_rect(State* L) {
    auto& canvas = require_canvas(L, 1);
    auto& image = canvas.getData();
    int x = tointeger(L, 2);
    int y = tointeger(L, 3);
    int w = tointeger(L, 4);
    int h = tointeger(L, 5);
    RGBA rgba = get_rgba(L, 6);
    image.drawRect(x, y, w, h, glm::ivec4 {rgba.r, rgba.g, rgba.b, rgba.a});
    return 0;
}

static int l_set_data(State* L) {
    auto& canvas = require_canvas(L, 1);
    auto& image = canvas.getData();
    auto data = image.getData();

    if (lua::isstring(L, 2)) {
        auto ptr = reinterpret_cast<ubyte*>(std::stoull(lua::tostring(L, 2)));
        int len = lua::touinteger(L, 3);
        if (len < image.getDataSize()) {
            throw std::runtime_error(
                "data size mismatch expected " +
                std::to_string(image.getDataSize()) + ", got " +
                std::to_string(len)
            );
        }
        std::memcpy(data, ptr, image.getDataSize());
        return 0;
    }
    int len = objlen(L, 2);
    if (len < image.getDataSize()) {
        throw std::runtime_error(
            "data size mismatch expected " +
            std::to_string(image.getDataSize()) + ", got " + std::to_string(len)
        );
    }
    for (size_t i = 0; i < len; i++) {
        rawgeti(L, i + 1, 2);
        data[i] = tointeger(L, -1);
        pop(L);
    }
    return 0;
}

static int l_get_data(State* L) {
    auto& canvas = require_canvas(L, 1);
    auto& image = canvas.getData();
    auto data = image.getData();
    return create_bytearray(L, data, image.getDataSize());
}

static int l_update(State* L) {
    if (auto canvas = touserdata<LuaCanvas>(L, 1)) {
        canvas->update();
    }
    return 0;
}

static int l_create_texture(State* L) {
    if (auto canvas = touserdata<LuaCanvas>(L, 1)) {
        if (!canvas->hasTexture()) {
            canvas->createTexture();
        }
        if (canvas->hasTexture()) {
            std::string name = require_string(L, 2);
            scripting::engine->getAssets()->store(canvas->shareTexture(), name);
        }
    }
    return 0;
}

static int l_mul(State* L) {
    auto canvas = touserdata<LuaCanvas>(L, 1);
    if (canvas == nullptr) {
        return 0;
    }
    if (lua::isnumber(L, 2)) {
        RGBA rgba = get_rgba(L, 2);
        canvas->getData().mulColor(glm::ivec4 {rgba.r, rgba.g, rgba.b, rgba.a});
    } else if (auto other = touserdata<LuaCanvas>(L, 2)) {
        canvas->getData().mulColor(other->getData());
    }
    return 0;
}

static int l_add(State* L) {
    auto canvas = touserdata<LuaCanvas>(L, 1);
    if (canvas == nullptr) {
        return 0;
    }
    if (lua::istable(L, 2)) {
        RGBA rgba = get_rgba(L, 2);
        canvas->getData().addColor(glm::ivec4 {rgba.r, rgba.g, rgba.b, rgba.a}, 1);
    } else if (auto other = touserdata<LuaCanvas>(L, 2)) {
        canvas->getData().addColor(other->getData(), 1);
    }
    return 0;
}

static int l_sub(State* L) {
    auto canvas = touserdata<LuaCanvas>(L, 1);
    if (canvas == nullptr) {
        return 0;
    }
    if (lua::istable(L, 2)) {
        RGBA rgba = get_rgba(L, 2);
        canvas->getData().addColor(glm::ivec4 {rgba.r, rgba.g, rgba.b, rgba.a}, -1);
    } else if (auto other = touserdata<LuaCanvas>(L, 2)) {
        canvas->getData().addColor(other->getData(), -1);
    }
    return 0;
}

static int l_encode(State* L) {
    auto canvas = touserdata<LuaCanvas>(L, 1);
    if (canvas == nullptr) {
        return 0;
    }
    auto format = imageio::ImageFileFormat::PNG;
    if (lua::isstring(L, 2)) {
        auto name = lua::require_string(L, 2);
        if (!imageio::ImageFileFormatMeta.getItem(name, format)) {
            throw std::runtime_error("unsupported image file format");
        }
    }

    auto buffer = imageio::encode(format, canvas->getData());
    return lua::create_bytearray(L, buffer.data(), buffer.size());
}

static std::unordered_map<std::string, lua_CFunction> methods {
    {"at", lua::wrap<l_at>},
    {"set", lua::wrap<l_set>},
    {"line", lua::wrap<l_line>},
    {"blit", lua::wrap<l_blit>},
    {"clear", lua::wrap<l_clear>},
    {"rect", lua::wrap<l_rect>},
    {"update", lua::wrap<l_update>},
    {"create_texture", lua::wrap<l_create_texture>},
    {"unbind_texture", lua::wrap<l_unbind_texture>},
    {"mul", lua::wrap<l_mul>},
    {"add", lua::wrap<l_add>},
    {"sub", lua::wrap<l_sub>},
    {"encode", lua::wrap<l_encode>},
    {"get_data", lua::wrap<l_get_data>},
    {"_set_data", lua::wrap<l_set_data>},
};

static int l_meta_index(State* L) {
    auto texture = touserdata<LuaCanvas>(L, 1);
    if (texture == nullptr) {
        return 0;
    }
    auto& data = texture->getData();
    if (isnumber(L, 2)) {
        if (auto pixel = get_at(data, static_cast<uint>(tointeger(L, 2)))) {
            return pushinteger(L, pixel->rgba);
        }
    }
    if (isstring(L, 2)) {
        auto name = tostring(L, 2);
        if (!strcmp(name, "width")) {
            return pushinteger(L, data.getWidth());
        }
        if (!strcmp(name, "height")) {
            return pushinteger(L, data.getHeight());
        }
        if (!strcmp(name, "set_data")) {
            return getglobal(L, "__vc_Canvas_set_data");
        }
        if (auto func = methods.find(tostring(L, 2)); func != methods.end()) {
            return pushcfunction(L, func->second);
        }
    }
    return 0;
}

static int l_meta_newindex(State* L) {
    auto texture = touserdata<LuaCanvas>(L, 1);
    if (texture == nullptr) {
        return 0;
    }
    auto& data = texture->getData();
    if (isnumber(L, 2) && isnumber(L, 3)) {
        if (auto pixel = get_at(data, static_cast<uint>(tointeger(L, 2)))) {
            pixel->rgba = static_cast<uint>(tointeger(L, 3));
        }
    }
    return 0;
}

static int l_meta_meta_call(lua::State* L) {
    auto size = glm::ivec2(tovec2(L, 2));
    if (size.x <= 0 || size.y <= 0) {
        throw std::runtime_error("size must be positive");
    }
    return newuserdata<LuaCanvas>(
        L,
        nullptr,
        std::make_shared<ImageData>(ImageFormat::RGBA8888, size.x, size.y)
    );
}

static int l_canvas_decode(lua::State* L) {
    auto bytes = bytearray_as_string(L, 1);
    auto formatName = require_lstring(L, 2);
    imageio::ImageFileFormat format;
    if (!imageio::ImageFileFormatMeta.getItem(formatName, format)) {
        throw std::runtime_error("unsupported image format");
    }
    return newuserdata<LuaCanvas>(
        L,
        nullptr,
        imageio::decode(
            format,
            {reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size()}
        )
    );
}

int LuaCanvas::createMetatable(State* L) {
    createtable(L, 0, 3);
    pushcfunction(L, lua::wrap<l_meta_index>);
    setfield(L, "__index");
    pushcfunction(L, lua::wrap<l_meta_newindex>);
    setfield(L, "__newindex");

    createtable(L, 0, 1);
    pushcfunction(L, lua::wrap<l_meta_meta_call>);
    setfield(L, "__call");
    setmetatable(L);

    pushcfunction(L, lua::wrap<l_canvas_decode>);
    setfield(L, "decode");
    return 1;
}
