#include "api_lua.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <sstream>

static int l_from_mat4(lua::State* L) {
    uint argc = lua::check_argc(L, 1, 2);
    auto matrix = lua::tomat4(L, 1);
    if (argc == 1) {
        return lua::pushquat(L, glm::toQuat(matrix));
    } else {
        return lua::setquat(L, 2, glm::toQuat(matrix));
    }
}

static int l_slerp(lua::State* L) {
    uint argc = lua::check_argc(L, 3, 4);
    auto quat1 = lua::toquat(L, 1);
    auto quat2 = lua::toquat(L, 2);
    float t = lua::tonumber(L, 3);
    if (argc == 3) {
        return lua::pushquat(L, glm::slerp(quat1, quat2, t));
    } else {
        return lua::setquat(L, 4, glm::slerp(quat1, quat2, t));
    }
}

static int l_tostring(lua::State* L) {
    lua::check_argc(L, 1);
    auto quat = lua::toquat(L, 1);

    std::stringstream ss;
    ss << "quat"
       << "{";
    for (int i = 0; i < 4; i++) {
        if (i > 0) {
            ss << ", ";
        }
        ss << quat[i];
    }
    ss << "}";
    return lua::pushstring(L, ss.str());
}

const luaL_Reg quatlib[] = {
    {"from_mat4", lua::wrap<l_from_mat4>},
    {"slerp", lua::wrap<l_slerp>},
    {"tostring", lua::wrap<l_tostring>},
    {nullptr, nullptr}
};
