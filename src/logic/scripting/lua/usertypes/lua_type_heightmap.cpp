#include "lua_type_heightmap.hpp"

#include "util/functional_util.hpp"
#define FNL_IMPL
#include "maths/FastNoiseLite.h"
#include "coders/imageio.hpp"
#include "io/util.hpp"
#include "graphics/core/ImageData.hpp"
#include "maths/Heightmap.hpp"
#include "engine/Engine.hpp"
#include "engine/EnginePaths.hpp"
#include "../lua_util.hpp"
#include "lua_type_heightmap.hpp"

#include <cstring>
#include <sstream>
#include <iomanip>
#include <filesystem>

using namespace lua;

LuaHeightmap::LuaHeightmap(const std::shared_ptr<Heightmap>& map)
 : map(map), noise(std::make_unique<fnl_state>(fnlCreateState())) {
}

LuaHeightmap::LuaHeightmap(uint width, uint height)
 : map(std::make_shared<Heightmap>(width, height)),
   noise(std::make_unique<fnl_state>(fnlCreateState())) 
{}

LuaHeightmap::~LuaHeightmap() {
}

void LuaHeightmap::setSeed(int64_t seed) {
    noise->seed = seed;
}

uint LuaHeightmap::getWidth() const {
    return map->getWidth();
}

uint LuaHeightmap::getHeight() const {
    return map->getHeight();
}

float* LuaHeightmap::getValues() {
    return map->getValues();
}

const float* LuaHeightmap::getValues() const {
    return map->getValues();
}

static int l_dump(lua::State* L) {
    if (auto heightmap = touserdata<LuaHeightmap>(L, 1)) {
        io::path file = require_string(L, 2);
        uint w = heightmap->getWidth();
        uint h = heightmap->getHeight();
        ImageData image(ImageFormat::rgb888, w, h);
        auto heights = heightmap->getValues();
        auto raster = image.getData();
        for (uint y = 0; y < h; y++) {
            for (uint x = 0; x < w; x++) {
                uint i = y * w + x;
                int val = heights[i] * 127 + 128;
                val = std::max(std::min(val, 255), 0);
                raster[i*3] = val;
                raster[i*3 + 1] = val;
                raster[i*3 + 2] = val;
            }
        }
        imageio::write(file, &image);
    }
    return 0;
}

static int l_at(lua::State* L) {
    if (auto heightmap = touserdata<LuaHeightmap>(L, 1)) {
        int x = lua::tointeger(L, 2);
        int y = lua::tointeger(L, 3);
        return lua::pushnumber(L, heightmap->getHeightmap()->get(x, y));
    }
    return 0;
}

template<fnl_noise_type noise_type>
static int l_noise(lua::State* L) {
    if (auto heightmap = touserdata<LuaHeightmap>(L, 1)) {
        uint w = heightmap->getWidth();
        uint h = heightmap->getHeight();
        auto heights = heightmap->getValues();
        auto noise = heightmap->getNoise();

        auto offset = tovec<2>(L, 2);

        float s = tonumber(L, 3);
        int octaves = 1;
        float multiplier = 1.0f;
        if (gettop(L) > 3) {
            octaves = tointeger(L, 4);
        }
        if (gettop(L) > 4) {
            multiplier = tonumber(L, 5);
        }
        const LuaHeightmap* shiftMapX = nullptr;
        const LuaHeightmap* shiftMapY = nullptr;
        if (gettop(L) > 5) {
            shiftMapX = touserdata<LuaHeightmap>(L, 6);
        }
        if (gettop(L) > 6) {
            shiftMapY = touserdata<LuaHeightmap>(L, 7);
        }
        noise->noise_type = noise_type;
        for (uint y = 0; y < h; y++) {
            for (uint x = 0; x < w; x++) {
                uint i = y * w + x;
                for (uint c = 0; c < octaves; c++) {
                    float m = s * (1 << c);
                    float value = heights[i];
                    float u = (x + offset.x) * m;
                    float v = (y + offset.y) * m;
                    if (shiftMapX) {
                        u += shiftMapX->getValues()[i];
                    }
                    if (shiftMapY) {
                        v += shiftMapY->getValues()[i];
                    }

                    value += fnlGetNoise2D(noise, u, v) /
                            static_cast<float>(1 << c) * multiplier;
                    heights[i] = value;
                }
            }
        }
    }
    return 0;
}

template<template<class> class Op>
static int l_binop_func(lua::State* L) {
    Op<float> op;
    if (auto heightmap = touserdata<LuaHeightmap>(L, 1)) {
        uint w = heightmap->getWidth();
        uint h = heightmap->getHeight();
        auto heights = heightmap->getValues();

        if (isnumber(L, 2)) {
            float scalar = tonumber(L, 2);
            for (uint y = 0; y < h; y++) {
                for (uint x = 0; x < w; x++) {
                    uint i = y * w + x;
                    heights[i] = op(heights[i], scalar);
                }
            }
        } else {
            auto map = touserdata<LuaHeightmap>(L, 2);
            for (uint y = 0; y < h; y++) {
                for (uint x = 0; x < w; x++) {
                    uint i = y * w + x;
                    heights[i] = op(heights[i], map->getValues()[i]);
                }
            }
        }
    }
    return 0;
}

static int l_mixin(lua::State* L) {
    if (auto heightmap = touserdata<LuaHeightmap>(L, 1)) {
        uint w = heightmap->getWidth();
        uint h = heightmap->getHeight();
        auto heights = heightmap->getValues();

        if (isnumber(L, 2)) {
            float scalar = tonumber(L, 2);
            if (isnumber(L, 3)) {
                float t = tonumber(L, 3);
                for (uint i = 0; i < w * h; i++) {
                    heights[i] = heights[i] * (1.0f - t) + scalar * t;
                }
            } else {
                auto tmap = touserdata<LuaHeightmap>(L, 3);
                auto tmapvalues = tmap->getValues();
                for (uint i = 0; i < w * h; i++) {
                    float t = tmapvalues[i];
                    heights[i] = heights[i] * (1.0f - t) + scalar * t;
                }
            }
        } else {
            auto map = touserdata<LuaHeightmap>(L, 2);
            auto mapvalues = map->getValues();
            if (isnumber(L, 3)) {
                float t = tonumber(L, 3);
                for (uint i = 0; i < w * h; i++) {
                    heights[i] = heights[i] * (1.0f - t) + mapvalues[i] * t;
                }
            } else {
                auto tmap = touserdata<LuaHeightmap>(L, 3);
                auto tmapvalues = tmap->getValues();
                for (uint i = 0; i < w * h; i++) {
                    float t = tmapvalues[i];
                    heights[i] = heights[i] * (1.0f - t) + mapvalues[i] * t;
                }
            }
        }
    }
    return 0;
}

template<template<class> class Op>
static int l_unaryop_func(lua::State* L) {
    Op<float> op;
    if (auto heightmap = touserdata<LuaHeightmap>(L, 1)) {
        uint w = heightmap->getWidth();
        uint h = heightmap->getHeight();
        auto heights = heightmap->getValues();
        for (uint y = 0; y < h; y++) {
            for (uint x = 0; x < w; x++) {
                uint i = y * w + x;
                heights[i] = op(heights[i]);
            }
        }
    }
    return 0;
}

static int l_resize(lua::State* L) {
    if (auto heightmap = touserdata<LuaHeightmap>(L, 1)) {
        uint width = touinteger(L, 2);
        uint height = touinteger(L, 3);
        auto interpName = tostring(L, 4);
        auto interpolation = InterpolationType::NEAREST;
        if (std::strcmp(interpName, "linear") == 0) {
            interpolation = InterpolationType::LINEAR;
        } else if (std::strcmp(interpName, "cubic") == 0) {
            interpolation = InterpolationType::CUBIC;
        }
        heightmap->getHeightmap()->resize(width, height, interpolation);
    }
    return 0;
}

static int l_crop(lua::State* L) {
    if (auto heightmap = touserdata<LuaHeightmap>(L, 1)) {
        uint srcX = touinteger(L, 2);
        uint srcY = touinteger(L, 3);

        uint dstWidth = touinteger(L, 4);
        uint dstHeight = touinteger(L, 5);

        heightmap->getHeightmap()->crop(srcX, srcY, dstWidth, dstHeight);
    }
    return 0;
}

static std::unordered_map<std::string, lua_CFunction> methods {
    {"dump", lua::wrap<l_dump>},
    {"noise", lua::wrap<l_noise<FNL_NOISE_OPENSIMPLEX2>>},
    {"cellnoise", lua::wrap<l_noise<FNL_NOISE_CELLULAR>>},
    {"pow", lua::wrap<l_binop_func<util::pow>>},
    {"add", lua::wrap<l_binop_func<std::plus>>},
    {"sub", lua::wrap<l_binop_func<std::minus>>},
    {"mul", lua::wrap<l_binop_func<std::multiplies>>},
    {"min", lua::wrap<l_binop_func<util::min>>},
    {"max", lua::wrap<l_binop_func<util::max>>},
    {"abs", lua::wrap<l_unaryop_func<util::abs>>},
    {"resize", lua::wrap<l_resize>},
    {"crop", lua::wrap<l_crop>},
    {"at", lua::wrap<l_at>},
    {"mixin", lua::wrap<l_mixin>},
};

static int l_meta_meta_call(lua::State* L) {
    auto width = tointeger(L, 2);
    auto height = tointeger(L, 3);
    if (width <= 0 || height <= 0) {
        throw std::runtime_error("width and height must be greater than 0");
    }
    return newuserdata<LuaHeightmap>(
        L, static_cast<uint>(width), static_cast<uint>(height)
    );
}

static int l_meta_index(lua::State* L) {
    auto map = touserdata<LuaHeightmap>(L, 1);
    if (map == nullptr) {
        return 0;
    }
    if (isstring(L, 2)) {
        auto fieldname = tostring(L, 2);
        if (!std::strcmp(fieldname, "width")) {
            return pushinteger(L, map->getWidth());
        } else if (!std::strcmp(fieldname, "height")) {
            return pushinteger(L, map->getHeight());
        } else {
            auto found = methods.find(tostring(L, 2));
            if (found != methods.end()) {
                return pushcfunction(L, found->second);
            }
        }
    }
    return 0;
}

static int l_meta_newindex(lua::State* L) {
    auto map = touserdata<LuaHeightmap>(L, 1);
    if (map == nullptr) {
        return 0;
    }
    if (isstring(L, 2)) {
        auto fieldname = tostring(L, 2);
        if (!std::strcmp(fieldname, "noiseSeed")) {
            map->setSeed(tointeger(L, 3));
        }
    }
    return 0;
}

static int l_meta_tostring(lua::State* L) {
    auto map = touserdata<LuaHeightmap>(L, 1);
    if (map == nullptr) {
        return 0;
    }

    std::stringstream stream;
    stream << std::hex << reinterpret_cast<ptrdiff_t>(map);
    auto ptrstr = stream.str();

    return pushstring(
        L, "Heightmap(" + std::to_string(map->getWidth()) + 
           "*" + std::to_string(map->getHeight()) + " at 0x" + ptrstr + ")"
    );
}

int LuaHeightmap::createMetatable(lua::State* L) {
    createtable(L, 0, 3);
    pushcfunction(L, lua::wrap<l_meta_tostring>);
    setfield(L, "__tostring");
    pushcfunction(L, lua::wrap<l_meta_index>);
    setfield(L, "__index");
    pushcfunction(L, lua::wrap<l_meta_newindex>);
    setfield(L, "__newindex");

    createtable(L, 0, 1);
    pushcfunction(L, lua::wrap<l_meta_meta_call>);
    setfield(L, "__call");
    setmetatable(L);
    return 1;
}
