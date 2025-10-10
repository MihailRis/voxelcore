#include "audio/audio.hpp"
#include "assets/Assets.hpp"
#include "engine/Engine.hpp"
#include "api_lua.hpp"

inline const char* DEFAULT_CHANNEL = "regular";

inline int extract_channel_index(lua::State* L, int idx) {
    const char* channel = DEFAULT_CHANNEL;
    if (!lua::isnoneornil(L, idx)) {
        channel = lua::tostring(L, idx);
    }
    int index = audio::get_channel_index(channel);
    if (index == 0) {
        return -1;
    }
    return index;
}

inline audio::speakerid_t play_sound(
    const char* name,
    bool relative,
    lua::Number x,
    lua::Number y,
    lua::Number z,
    lua::Number volume,
    lua::Number pitch,
    bool loop,
    int channel
) {
    if (channel == -1) {
        return 0;
    }
    auto assets = scripting::engine->getAssets();
    if (assets == nullptr) {
        return 0;
    }
    auto sound = assets->get<audio::Sound>(name);
    if (sound == nullptr) {
        return 0;
    }
    return audio::play(
        sound,
        glm::vec3(
            static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)
        ),
        relative,
        volume,
        pitch,
        loop,
        audio::PRIORITY_NORMAL,
        channel
    );
}

inline audio::speakerid_t play_stream(
    const char* filename,
    bool relative,
    lua::Number x,
    lua::Number y,
    lua::Number z,
    lua::Number volume,
    lua::Number pitch,
    bool loop,
    int channel
) {
    if (channel == -1) {
        return 0;
    }
    io::path file;
    if (std::strchr(filename, ':')) {
        file = std::string(filename);
    } else {
        const auto& paths = scripting::engine->getResPaths();
        file = paths.find(filename);
    }
    return audio::play_stream(
        file,
        glm::vec3(
            static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)
        ),
        relative,
        volume,
        pitch,
        loop,
        channel
    );
}

/// @brief audio.play_stream(
///            name: string,
///            x: number,
///            y: number,
///            z: number,
///            volume: number,
///            pitch: number,
///            channel: string = "regular",
///            loop: bool = false)
static int l_audio_play_stream(lua::State* L) {
    return lua::pushinteger(
        L,
        static_cast<lua::Integer>(play_stream(
            lua::tostring(L, 1),
            false,
            lua::tonumber(L, 2),
            lua::tonumber(L, 3),
            lua::tonumber(L, 4),
            lua::tonumber(L, 5),
            lua::tonumber(L, 6),
            lua::toboolean(L, 8),
            extract_channel_index(L, 7)
        ))
    );
}

/// @brief audio.play_stream_2d(
///            name: string,
///            volume: number,
///            pitch: number,
///            channel: string = "regular",
///            loop: bool = false)
static int l_audio_play_stream_2d(lua::State* L) {
    return lua::pushinteger(
        L,
        static_cast<lua::Integer>(play_stream(
            lua::tostring(L, 1),
            true,
            0.0,
            0.0,
            0.0,
            lua::tonumber(L, 2),
            lua::tonumber(L, 3),
            lua::toboolean(L, 5),
            extract_channel_index(L, 4)
        ))
    );
}

/// @brief audio.play_sound(
///            name: string,
///            x: number,
///            y: number,
///            z: number,
///            volume: number,
///            pitch: number,
///            channel: string = "regular",
///            loop: bool = false)
static int l_audio_play_sound(lua::State* L) {
    return lua::pushinteger(
        L,
        static_cast<lua::Integer>(play_sound(
            lua::tostring(L, 1),
            false,
            lua::tonumber(L, 2),
            lua::tonumber(L, 3),
            lua::tonumber(L, 4),
            lua::tonumber(L, 5),
            lua::tonumber(L, 6),
            lua::toboolean(L, 8),
            extract_channel_index(L, 7)
        ))
    );
}

/// @brief audio.play_sound_2d(
///            name: string,
///            volume: number,
///            pitch: number,
///            channel: string = "regular",
///            loop: bool = false)
static int l_audio_play_sound_2d(lua::State* L) {
    return lua::pushinteger(
        L,
        static_cast<lua::Integer>(play_sound(
            lua::tostring(L, 1),
            true,
            0.0,
            0.0,
            0.0,
            lua::tonumber(L, 2),
            lua::tonumber(L, 3),
            lua::toboolean(L, 5),
            extract_channel_index(L, 4)
        ))
    );
}

/// @brief audio.stop(speakerid: integer) -> nil
static int l_audio_stop(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        speaker->stop();
    }
    return 0;
}

/// @brief audio.pause(speakerid: integer) -> nil
static int l_audio_pause(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        speaker->pause();
    }
    return 0;
}

/// @brief audio.resume(speakerid: integer) -> nil
static int l_audio_resume(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr && speaker->isPaused()) {
        speaker->play();
    }
    return 0;
}

/// @brief audio.set_loop(speakerid: integer, value: bool) -> nil
static int l_audio_set_loop(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        speaker->setLoop(lua::toboolean(L, 2));
    }
    return 0;
}

/// @brief audio.set_volume(speakerid: integer, value: number) -> nil
static int l_audio_set_volume(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        speaker->setVolume(static_cast<float>(lua::tonumber(L, 2)));
    }
    return 0;
}

/// @brief audio.set_pitch(speakerid: integer, value: number) -> nil
static int l_audio_set_pitch(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        speaker->setPitch(static_cast<float>(lua::tonumber(L, 2)));
    }
    return 0;
}

/// @brief audio.set_time(speakerid: integer, value: number) -> nil
static int l_audio_set_time(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        speaker->setTime(static_cast<audio::duration_t>(lua::tonumber(L, 2)));
    }
    return 0;
}

/// @brief audio.set_position(speakerid: integer, x: number, y: number, z:
/// number) -> nil
static int l_audio_set_position(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        auto x = lua::tonumber(L, 2);
        auto y = lua::tonumber(L, 3);
        auto z = lua::tonumber(L, 4);
        speaker->setPosition(glm::vec3(
            static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)
        ));
    }
    return 0;
}

/// @brief audio.set_velocity(speakerid: integer, x: number, y: number, z:
/// number) -> nil
static int l_audio_set_velocity(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        auto x = lua::tonumber(L, 2);
        auto y = lua::tonumber(L, 3);
        auto z = lua::tonumber(L, 4);
        speaker->setVelocity(glm::vec3(
            static_cast<float>(x), static_cast<float>(y), static_cast<float>(z)
        ));
    }
    return 0;
}

/// @brief audio.is_playing(speakerid: integer) -> bool
static int l_audio_is_playing(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        return lua::pushboolean(L, speaker->isPlaying());
    }
    return lua::pushboolean(L, false);
}

/// @brief audio.is_paused(speakerid: integer) -> bool
static int l_audio_is_paused(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        return lua::pushboolean(L, speaker->isPaused());
    }
    return lua::pushboolean(L, false);
}

/// @brief audio.is_loop(speakerid: integer) -> bool
static int l_audio_is_loop(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        return lua::pushboolean(L, speaker->isLoop());
    }
    return lua::pushboolean(L, false);
}

/// @brief audio.get_volume(speakerid: integer) -> number
static int l_audio_get_volume(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        return lua::pushnumber(L, speaker->getVolume());
    }
    return lua::pushnumber(L, 0.0);
}

/// @brief audio.get_pitch(speakerid: integer) -> number
static int l_audio_get_pitch(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        return lua::pushnumber(L, speaker->getPitch());
    }
    return lua::pushnumber(L, 1.0);
}

/// @brief audio.get_time(speakerid: integer) -> number
static int l_audio_get_time(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        return lua::pushnumber(L, speaker->getTime());
    }
    return lua::pushnumber(L, 0.0);
}

/// @brief audio.get_duration(speakerid: integer) -> number
static int l_audio_get_duration(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        return lua::pushnumber(L, speaker->getDuration());
    }
    return lua::pushnumber(L, 0.0);
}

/// @brief audio.get_position(speakerid: integer) -> number, number, number
static int l_audio_get_position(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        return lua::pushvec_stack(L, speaker->getPosition());
    }
    return 0;
}

/// @brief audio.get_velocity(speakerid: integer) -> number, number, number
static int l_audio_get_velocity(lua::State* L) {
    auto speaker = audio::get_speaker(lua::tointeger(L, 1));
    if (speaker != nullptr) {
        return lua::pushvec_stack(L, speaker->getVelocity());
    }
    return 0;
}

// @brief audio.count_speakers() -> integer
static int l_audio_count_speakers(lua::State* L) {
    return lua::pushinteger(L, audio::count_speakers());
}

// @brief audio.count_streams() -> integer
static int l_audio_count_streams(lua::State* L) {
    return lua::pushinteger(L, audio::count_streams());
}

const luaL_Reg audiolib[] = {
    {"play_sound", lua::wrap<l_audio_play_sound>},
    {"play_sound_2d", lua::wrap<l_audio_play_sound_2d>},
    {"play_stream", lua::wrap<l_audio_play_stream>},
    {"play_stream_2d", lua::wrap<l_audio_play_stream_2d>},
    {"stop", lua::wrap<l_audio_stop>},
    {"pause", lua::wrap<l_audio_pause>},
    {"resume", lua::wrap<l_audio_resume>},
    {"set_loop", lua::wrap<l_audio_set_loop>},
    {"set_volume", lua::wrap<l_audio_set_volume>},
    {"set_pitch", lua::wrap<l_audio_set_pitch>},
    {"set_time", lua::wrap<l_audio_set_time>},
    {"set_position", lua::wrap<l_audio_set_position>},
    {"set_velocity", lua::wrap<l_audio_set_velocity>},
    {"is_playing", lua::wrap<l_audio_is_playing>},
    {"is_paused", lua::wrap<l_audio_is_paused>},
    {"is_loop", lua::wrap<l_audio_is_loop>},
    {"get_volume", lua::wrap<l_audio_get_volume>},
    {"get_pitch", lua::wrap<l_audio_get_pitch>},
    {"get_time", lua::wrap<l_audio_get_time>},
    {"get_duration", lua::wrap<l_audio_get_duration>},
    {"get_position", lua::wrap<l_audio_get_position>},
    {"get_velocity", lua::wrap<l_audio_get_velocity>},
    {"count_speakers", lua::wrap<l_audio_count_speakers>},
    {"count_streams", lua::wrap<l_audio_count_streams>},
    {nullptr, nullptr}
};
