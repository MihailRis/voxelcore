#include "../lua_util.hpp"
#include "lua_type_pcmstream.hpp"
#include "assets/Assets.hpp"
#include "audio/MemoryPCMStream.hpp"
#include "engine/Engine.hpp"

using namespace lua;
using namespace audio;
using namespace scripting;

LuaPCMStream::LuaPCMStream(std::shared_ptr<audio::MemoryPCMStream>&& stream)
    : stream(std::move(stream)) {
}

LuaPCMStream::~LuaPCMStream() = default;

const std::shared_ptr<audio::MemoryPCMStream>& LuaPCMStream::getStream() const {
    return stream;
}

static int l_feed(lua::State* L) {
    auto stream = touserdata<LuaPCMStream>(L, 1);
    if (stream == nullptr) {
        return 0;
    }
    auto bytes = bytearray_as_string(L, 2);
    stream->getStream()->feed(
        {reinterpret_cast<const ubyte*>(bytes.data()), bytes.size()}
    );
    return 0;
}

static int l_share(lua::State* L) {
    auto stream = touserdata<LuaPCMStream>(L, 1);
    if (stream == nullptr) {
        return 0;
    }
    auto alias = require_lstring(L, 2);
    if (engine->isHeadless()) {
        return 0;
    }
    auto assets = engine->getAssets();
    assets->store<PCMStream>(stream->getStream(), std::string(alias));
    return 0;
}

static int l_create_sound(lua::State* L) {
    auto stream = touserdata<LuaPCMStream>(L, 1);
    if (stream == nullptr) {
        return 0;
    }
    auto alias = require_lstring(L, 2);
    auto memoryStream = stream->getStream();

    std::vector<char> buffer(memoryStream->available());
    memoryStream->readFully(buffer.data(), buffer.size(), true);
    
    auto pcm = std::make_shared<PCM>(
        std::move(buffer),
        0,
        memoryStream->getChannels(),
        static_cast<uint8_t>(memoryStream->getBitsPerSample()),
        memoryStream->getSampleRate(),
        memoryStream->isSeekable()
    );
    auto sound = audio::create_sound(std::move(pcm), true);
    auto assets = engine->getAssets();
    assets->store<audio::Sound>(std::move(sound), std::string(alias));
    return 0;
}

static std::unordered_map<std::string, lua_CFunction> methods {
    {"feed", lua::wrap<l_feed>},
    {"share", lua::wrap<l_share>},
    {"create_sound", lua::wrap<l_create_sound>},
};

static int l_meta_meta_call(lua::State* L) {
    auto sampleRate = touinteger(L, 2);
    auto channels = touinteger(L, 3);
    auto bitsPerSample = touinteger(L, 4);
    auto stream =
        std::make_shared<MemoryPCMStream>(sampleRate, channels, bitsPerSample);
    return newuserdata<LuaPCMStream>(L, std::move(stream));
}

static int l_meta_tostring(lua::State* L) {
    return pushstring(L, "PCMStream");
}

static int l_meta_index(lua::State* L) {
    auto stream = touserdata<LuaPCMStream>(L, 1);
    if (stream == nullptr) {
        return 0;
    }
    if (isstring(L, 2)) {
        auto found = methods.find(tostring(L, 2));
        if (found != methods.end()) {
            return pushcfunction(L, found->second);
        }
    }
    return 0;
}

int LuaPCMStream::createMetatable(lua::State* L) {
    createtable(L, 0, 3);
    pushcfunction(L, lua::wrap<l_meta_tostring>);
    setfield(L, "__tostring");
    pushcfunction(L, lua::wrap<l_meta_index>);
    setfield(L, "__index");

    createtable(L, 0, 1);
    pushcfunction(L, lua::wrap<l_meta_meta_call>);
    setfield(L, "__call");
    setmetatable(L);
    return 1;
}
