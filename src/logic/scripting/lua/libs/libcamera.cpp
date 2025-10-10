#include <glm/ext.hpp>

#include "content/Content.hpp"
#include "window/Camera.hpp"
#include "world/Level.hpp"
#include "api_lua.hpp"

using namespace scripting;

template <int (*getterfunc)(lua::State*, const Camera&)>
static int l_camera_getter(lua::State* L) {
    size_t index = static_cast<size_t>(lua::tointeger(L, 1));
    return getterfunc(L, *level->cameras.at(index));
}

template <void (*setterfunc)(lua::State*, Camera&, int)>
static int l_camera_setter(lua::State* L) {
    size_t index = static_cast<size_t>(lua::tointeger(L, 1));
    setterfunc(L, *level->cameras.at(index), 2);
    return 0;
}

static int l_index(lua::State* L) {
    auto name = lua::require_string(L, 1);
    auto& indices = content->getIndices(ResourceType::CAMERA);
    return lua::pushinteger(L, indices.indexOf(name));
}

static int l_name(lua::State* L) {
    size_t index = static_cast<size_t>(lua::tointeger(L, 1));
    auto& indices = content->getIndices(ResourceType::CAMERA);
    return lua::pushstring(L, indices.getName(index));
}

static int getter_pos(lua::State* L, const Camera& camera) {
    return lua::pushvec3(L, camera.position);
}

static void setter_pos(lua::State* L, Camera& camera, int idx) {
    camera.position = lua::tovec<3>(L, idx);
}

static int getter_rot(lua::State* L, const Camera& camera) {
    return lua::pushmat4(L, camera.rotation);
}

static void setter_rot(lua::State* L, Camera& camera, int idx) {
    camera.rotation = lua::tomat4(L, idx);
    camera.updateVectors();
}

static int getter_zoom(lua::State* L, const Camera& camera) {
    return lua::pushnumber(L, camera.zoom);
}
static void setter_zoom(lua::State* L, Camera& camera, int idx) {
    camera.zoom = lua::tonumber(L, idx);
}

static int getter_fov(lua::State* L, const Camera& camera) {
    return lua::pushnumber(L, glm::degrees(camera.getFov()));
}
static void setter_fov(lua::State* L, Camera& camera, int idx) {
    camera.setFov(glm::radians(lua::tonumber(L, idx)));
}

static int getter_perspective(lua::State* L, const Camera& camera) {
    return lua::pushboolean(L, camera.perspective);
}
static void setter_perspective(lua::State* L, Camera& camera, int idx) {
    camera.perspective = lua::toboolean(L, idx);
}

static int getter_flipped(lua::State* L, const Camera& camera) {
    return lua::pushboolean(L, camera.flipped);
}
static void setter_flipped(lua::State* L, Camera& camera, int idx) {
    camera.flipped = lua::toboolean(L, idx);
}

static int getter_front(lua::State* L, const Camera& camera) {
    return lua::pushvec3(L, camera.front);
}
static int getter_right(lua::State* L, const Camera& camera) {
    return lua::pushvec3(L, camera.right);
}
static int getter_up(lua::State* L, const Camera& camera) {
    return lua::pushvec3(L, camera.up);
}

static int l_look_at(lua::State* L) {
    size_t index = static_cast<size_t>(lua::tointeger(L, 1));
    auto& camera = *level->cameras.at(index);
    auto center = lua::tovec<3>(L, 2);
    auto matrix = glm::inverse(
        glm::lookAt(glm::vec3(), center - camera.position, glm::vec3(0, 1, 0))
    );
    if (lua::isnumber(L, 3)) {
        matrix = glm::mat4_cast(glm::slerp(
            glm::quat(camera.rotation),
            glm::quat(matrix),
            static_cast<float>(lua::tonumber(L, 3))
        ));
    }
    camera.rotation = matrix;
    camera.updateVectors();
    return 0;
}

const luaL_Reg cameralib[] = {
    {"index", lua::wrap<l_index>},
    {"name", lua::wrap<l_name>},
    {"get_pos", lua::wrap<l_camera_getter<getter_pos>>},
    {"set_pos", lua::wrap<l_camera_setter<setter_pos>>},
    {"get_rot", lua::wrap<l_camera_getter<getter_rot>>},
    {"set_rot", lua::wrap<l_camera_setter<setter_rot>>},
    {"get_zoom", lua::wrap<l_camera_getter<getter_zoom>>},
    {"set_zoom", lua::wrap<l_camera_setter<setter_zoom>>},
    {"get_fov", lua::wrap<l_camera_getter<getter_fov>>},
    {"set_fov", lua::wrap<l_camera_setter<setter_fov>>},
    {"is_perspective", lua::wrap<l_camera_getter<getter_perspective>>},
    {"set_perspective", lua::wrap<l_camera_setter<setter_perspective>>},
    {"is_flipped", lua::wrap<l_camera_getter<getter_flipped>>},
    {"set_flipped", lua::wrap<l_camera_setter<setter_flipped>>},
    {"get_front", lua::wrap<l_camera_getter<getter_front>>},
    {"get_right", lua::wrap<l_camera_getter<getter_right>>},
    {"get_up", lua::wrap<l_camera_getter<getter_up>>},
    {"look_at", lua::wrap<l_look_at>},
    {nullptr, nullptr}
};
