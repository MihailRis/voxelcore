#include "libentity.hpp"

#include "animation/rigging.hpp"
#include "engine/Engine.hpp"
#include "graphics/render/WorldRenderer.hpp"
#include "graphics/render/NamedSkeletons.hpp"

#include <algorithm>
#include <glm/gtc/matrix_transform.hpp>

namespace scripting {
    extern WorldRenderer* renderer;
}

static int index_range_check(
    const rigging::Skeleton& skeleton, lua::Integer index
) {
    if (static_cast<size_t>(index) >= skeleton.pose.matrices.size()) {
        throw std::runtime_error(
            "index out of range [0, " +
            std::to_string(skeleton.pose.matrices.size()) + "]"
        );
    }
    return static_cast<int>(index);
}

static rigging::Skeleton* get_skeleton(lua::State* L) {
    if (scripting::engine->isHeadless()) {
        return nullptr;
    }
    if (lua::isstring(L, 1)) {
        return scripting::renderer->skeletons->getSkeleton(lua::tostring(L, 1));
    }
    if (auto entity = get_entity(L, 1)) {
        return entity->getSkeleton();
    }
    return nullptr;
}

static int l_get_model(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        if (skeleton->config == nullptr) {
            return 0;
        }
        auto& rigConfig = *skeleton->config;
        auto index = index_range_check(*skeleton, lua::tointeger(L, 2));
        const auto& modelOverride = skeleton->modelOverrides[index];
        if (modelOverride.model.expired()) {
            return lua::pushstring(L, modelOverride.name);
        }
        return lua::pushstring(L, rigConfig.getBones()[index]->model.name);
    }
    return 0;
}

static int l_set_model(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        auto index = index_range_check(*skeleton, lua::tointeger(L, 2));
        auto& modelOverride = skeleton->modelOverrides[index];
        if (lua::isnoneornil(L, 3)) {
            modelOverride = {"", {}, true};
        } else {
            modelOverride = {lua::require_string(L, 3), {}, true};
        }
    }
    return 0;
}

static int l_get_matrix(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        auto index = index_range_check(*skeleton, lua::tointeger(L, 2));
        if (lua::istable(L, 3)) {
            lua::setmat4(L, 3, skeleton->pose.matrices[index]);
        } else {
            return lua::pushmat4(L, skeleton->pose.matrices[index]);
        }
    }
    return 0;
}

static int l_set_matrix(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        if (lua::isnoneornil(L, 2)) {
            return 0;
        }
        auto index = index_range_check(*skeleton, lua::tointeger(L, 2));
        skeleton->pose.matrices[index] = lua::tomat4(L, 3);
    }
    return 0;
}

static int l_reset_pose(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        auto& pose = skeleton->pose;
        std::fill(pose.matrices.begin(), pose.matrices.end(), glm::mat4(1.0f));
    }
    return 0;
}

static int l_get_texture(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        const auto& found = skeleton->textures.find(lua::require_string(L, 2));
        if (found != skeleton->textures.end()) {
            return lua::pushstring(L, found->second);
        }
    }
    return 0;
}

static int l_set_texture(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        skeleton->textures[lua::require_string(L, 2)] =
            lua::require_string(L, 3);
    }
    return 0;
}

static int l_index(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        if (skeleton->config == nullptr) {
            return 0;
        }
        if (auto bone = skeleton->config->find(lua::require_string(L, 2))) {
            return lua::pushinteger(L, bone->getIndex());
        }
    }
    return 0;
}

static int l_is_visible(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        if (!lua::isnoneornil(L, 2)) {
            auto index = index_range_check(*skeleton, lua::tointeger(L, 2));
            return lua::pushboolean(L, skeleton->flags.at(index).visible);
        }
        return lua::pushboolean(L, skeleton->visible);
    }
    return 0;
}

static int l_set_visible(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        if (!lua::isnoneornil(L, 3)) {
            auto index = index_range_check(*skeleton, lua::tointeger(L, 2));
            skeleton->flags.at(index).visible = lua::toboolean(L, 3);
        } else {
            skeleton->visible = lua::toboolean(L, 2);
        }
    }
    return 0;
}

static glm::vec4 to_color(lua::State* L, int idx) {
    if (lua::objlen(L, idx) > 3) {
        return lua::tovec4(L, idx);
    }
    return glm::vec4(lua::tovec3(L, idx), 1.0f);
}

static int l_get_color(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        if (lua::isnumber(L, 2)) {
            auto index = index_range_check(*skeleton, lua::tointeger(L, 2));
            return lua::pushvec(L, skeleton->boneTints[index]);
        }
        return lua::pushvec(L, skeleton->tint);
    }
    return 0;
}

static int l_set_color(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        auto color = to_color(L, 2);
        if (lua::isnumber(L, 3)) {
            auto index = index_range_check(*skeleton, lua::tointeger(L, 3));
            skeleton->boneTints[index] = color;
        } else {
            skeleton->tint = color;
        }
    }
    return 0;
}

static int l_is_interpolated(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        return lua::pushboolean(L, skeleton->interpolation.isEnabled());
    }
    return 0;
}

static int l_set_interpolated(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        skeleton->interpolation.setEnabled(lua::toboolean(L, 2));
    }
    return 0;
}

static int l_exists(lua::State* L) {
    return lua::pushboolean(L, get_skeleton(L));
}

namespace {
    struct NativeTrackBone {
        std::string name;
        bool translate = false;
        int translation[3] {};
        std::vector<std::pair<int, int>> rotations;
        bool scale = false;
        int scaling[3] {};
    };

    struct NativeTrack {
        std::string name;
        std::vector<NativeTrackBone> bones;
        int valuesCount = 0;
        std::weak_ptr<const rigging::SkeletonConfig> config;
        std::vector<int> indices;
    };

    std::vector<NativeTrack> native_tracks;

    constexpr int MAX_TRACK_VALUES = 256;
}

static void read_indices(lua::State* L, int idx, const char* field, int* dst) {
    if (!lua::istable(L, -1)) {
        return;
    }
    for (int i = 0; i < 3; i++) {
        lua::rawgeti(L, i + 1);
        dst[i] = static_cast<int>(lua::tointeger(L, -1));
        lua::pop(L);
    }
}

static int l_register_track(lua::State* L) {
    NativeTrack track;
    track.name = lua::require_string(L, 1);
    if (!lua::istable(L, 2)) {
        throw std::runtime_error("layout table expected");
    }
    int maxValue = 0;
    size_t count = lua::objlen(L, 2);
    for (size_t i = 1; i <= count; i++) {
        lua::rawgeti(L, static_cast<int>(i), 2);
        int bone = lua::gettop(L);
        NativeTrackBone entry;
        lua::requirefield(L, "name", bone);
        entry.name = lua::require_string(L, -1);
        lua::pop(L);

        if (lua::getfield(L, "t", bone)) {
            entry.translate = lua::istable(L, -1);
            read_indices(L, bone, "t", entry.translation);
            lua::pop(L);
        }

        if (lua::getfield(L, "s", bone)) {
            entry.scale = lua::istable(L, -1);
            read_indices(L, bone, "s", entry.scaling);
            lua::pop(L);
        }

        if (lua::getfield(L, "r", bone)) {
            if (lua::istable(L, -1)) {
                size_t n = lua::objlen(L, -1);
                for (size_t j = 1; j + 1 <= n; j += 2) {
                    lua::rawgeti(L, static_cast<int>(j), -1);
                    int axis = static_cast<int>(lua::tointeger(L, -1));
                    lua::rawgeti(L, static_cast<int>(j + 1), -2);
                    int value = static_cast<int>(lua::tointeger(L, -1));
                    lua::pop(L, 2);
                    if (axis < 0 || axis > 2) {
                        throw std::runtime_error("invalid rotation axis");
                    }
                    entry.rotations.emplace_back(axis, value);
                }
            }
            lua::pop(L);
        }
        for (int v : entry.translation) {
            maxValue = std::max(maxValue, v);
        }
        for (int v : entry.scaling) {
            maxValue = std::max(maxValue, v);
        }
        for (auto [_, v] : entry.rotations) {
            maxValue = std::max(maxValue, v);
        }
        track.bones.push_back(std::move(entry));
        lua::pop(L);
    }
    track.valuesCount = maxValue;
    for (size_t i = 0; i < native_tracks.size(); i++) {
        if (native_tracks[i].name == track.name) {
            native_tracks[i] = std::move(track);
            return lua::pushinteger(L, static_cast<lua::Integer>(i));
        }
    }
    native_tracks.push_back(std::move(track));
    return lua::pushinteger(L, native_tracks.size() - 1);
}

static int l_apply_track(lua::State* L) {
    auto skeleton = get_skeleton(L);
    if (skeleton == nullptr || skeleton->config == nullptr) {
        return 0;
    }
    auto id = lua::tointeger(L, 2);
    if (id < 0 || static_cast<size_t>(id) >= native_tracks.size()) {
        throw std::runtime_error("invalid track id");
    }
    auto& track = native_tracks[id];
    const auto& config = skeleton->config;
    if (track.config.owner_before(config) || config.owner_before(track.config) ||
        track.config.expired() || track.indices.size() != track.bones.size()) {
        track.config = config;
        track.indices.clear();
        for (const auto& bone : track.bones) {
            auto found = config->find(bone.name);
            track.indices.push_back(found ? static_cast<int>(found->getIndex()) : -1);
        }
    }
    float values[MAX_TRACK_VALUES + 1] {};
    int top = lua::gettop(L);
    int count = std::min({track.valuesCount, top - 2, MAX_TRACK_VALUES});
    for (int i = 1; i <= count; i++) {
        values[i] = static_cast<float>(lua::tonumber(L, i + 2));
    }
    static const glm::vec3 AXES[] {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    auto value = [&values](int index, float fallback) {
        return index > 0 && index <= MAX_TRACK_VALUES ? values[index] : fallback;
    };
    auto& matrices = skeleton->pose.matrices;
    for (size_t i = 0; i < track.bones.size(); i++) {
        int index = track.indices[i];
        if (index < 0 || static_cast<size_t>(index) >= matrices.size()) {
            continue;
        }
        const auto& bone = track.bones[i];
        auto& matrix = matrices[index];
        if (bone.translate) {
            matrix = glm::translate(matrix, glm::vec3(
                value(bone.translation[0], 0.0f),
                value(bone.translation[1], 0.0f),
                value(bone.translation[2], 0.0f)
            ));
        }
        for (const auto& [axis, v] : bone.rotations) {
            matrix = glm::rotate(matrix, glm::radians(value(v, 0.0f)), AXES[axis]);
        }
        if (bone.scale) {
            matrix = glm::scale(matrix, glm::vec3(
                value(bone.scaling[0], 1.0f),
                value(bone.scaling[1], 1.0f),
                value(bone.scaling[2], 1.0f)
            ));
        }
    }
    return 0;
}

static int l_is_culled(lua::State* L) {
    if (auto skeleton = get_skeleton(L)) {
        return lua::pushboolean(L, skeleton->culled);
    }
    return lua::pushboolean(L, false);
}

const luaL_Reg skeletonlib[] = {
    {"get_model", lua::wrap<l_get_model>},
    {"set_model", lua::wrap<l_set_model>},
    {"get_matrix", lua::wrap<l_get_matrix>},
    {"set_matrix", lua::wrap<l_set_matrix>},
    {"reset_pose", lua::wrap<l_reset_pose>},
    {"get_texture", lua::wrap<l_get_texture>},
    {"set_texture", lua::wrap<l_set_texture>},
    {"index", lua::wrap<l_index>},
    {"is_visible", lua::wrap<l_is_visible>},
    {"set_visible", lua::wrap<l_set_visible>},
    {"get_color", lua::wrap<l_get_color>},
    {"set_color", lua::wrap<l_set_color>},
    {"is_interpolated", lua::wrap<l_is_interpolated>},
    {"set_interpolated", lua::wrap<l_set_interpolated>},
    {"exists", lua::wrap<l_exists>},
    {"is_culled", lua::wrap<l_is_culled>},
    {"register_track", lua::wrap<l_register_track>},
    {"apply_track", lua::wrap<l_apply_track>},
    {nullptr, nullptr}
};
