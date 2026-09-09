#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/random.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <sstream>

#include "api_lua.hpp"

template <typename T>
inline T angle(glm::vec<2, T> vec) {
    auto val = std::atan2(vec.y, vec.x);
    if (val < 0.0) {
        return val + glm::two_pi<double>();
    }
    return val;
}

template <int n>
static int l_mix(lua::State* L) {
    uint argc = lua::check_argc(L, 3, 4);
    auto a = lua::tovec<n, number_t>(L, 1);
    auto b = lua::tovec<n, number_t>(L, 2);
    auto t = lua::tonumber(L, 3);

    if (argc == 3) {
        return lua::pushvec(L, a * (1.0 - t) + b * t);
    } else {
        return lua::setvec(L, 4, a * (1.0 - t) + b * t);
    }
}

template <int n, template <class> class Op>
static int l_binop(lua::State* L) {
    uint argc = lua::check_argc(L, 2, 3);
    auto a = lua::tovec<n, number_t>(L, 1);

    if (lua::isnumber(L, 2)) {  // scalar second operand overload
        auto b = lua::tonumber(L, 2);
        Op op;
        if (argc == 2) {
            lua::createtable(L, n, 0);
            for (uint i = 0; i < n; i++) {
                lua::pushnumber(L, op(a[i], b));
                lua::rawseti(L, i + 1);
            }
            return 1;
        } else {
            return lua::setvec(L, 3, op(a, glm::vec<n, number_t>(b)));
        }
    } else {
        auto b = lua::tovec<n, number_t>(L, 2);
        Op op;
        if (argc == 2) {
            lua::createtable(L, n, 0);
            for (uint i = 0; i < n; i++) {
                lua::pushnumber(L, op(a[i], b[i]));
                lua::rawseti(L, i + 1);
            }
            return 1;
        } else {
            return lua::setvec(L, 3, op(a, b));
        }
    }
}

template <int n, glm::vec<n, number_t> (*func)(const glm::vec<n, number_t>&)>
static int l_unaryop(lua::State* L) {
    uint argc = lua::check_argc(L, 1, 2);
    auto vec = func(lua::tovec<n, number_t>(L, 1));
    switch (argc) {
        case 1:
            lua::createtable(L, n, 0);
            for (uint i = 0; i < n; i++) {
                lua::pushnumber(L, vec[i]);
                lua::rawseti(L, i + 1);
            }
            return 1;
        case 2:
            return lua::setvec(L, 2, vec);
    }
    return 0;
}

template <int n, number_t (*func)(const glm::vec<n, number_t>&)>
static int l_scalar_op(lua::State* L) {
    lua::check_argc(L, 1);
    auto vec = lua::tovec<n, number_t>(L, 1);
    return lua::pushnumber(L, func(vec));
}

template <int n>
static int l_distance(lua::State* L) {
    lua::check_argc(L, 2);
    auto a = lua::tovec<n, number_t>(L, 1);
    auto b = lua::tovec<n, number_t>(L, 2);
    return lua::pushnumber(L,glm::distance(a, b));
}

template <int n>
static int l_pow(lua::State* L) {
    uint argc = lua::check_argc(L, 2, 3);
    auto a = lua::tovec<n, number_t>(L, 1);

    if (lua::isnumber(L, 2)) {
        auto b = lua::tonumber(L, 2);
        if (argc == 2) {
            lua::createtable(L, n, 0);
            for (uint i = 0; i < n; i++) {
                lua::pushnumber(L, pow(a[i], b));
                lua::rawseti(L, i + 1);
            }
            return 1;
        } else {
            return lua::setvec(L, 3, pow(a, glm::vec<n, number_t>(b)));
        }
    } else {
        auto b = lua::tovec<n, number_t>(L, 2);
        if (argc == 2) {
            lua::createtable(L, n, 0);
            for (uint i = 0; i < n; i++) {
                lua::pushnumber(L, pow(a[i], b[i]));
                lua::rawseti(L, i + 1);
            }
            return 1;
        } else {
            return lua::setvec(L, 3, pow(a, b));
        }
    }
}

template <int n>
static int l_dot(lua::State* L) {
    lua::check_argc(L, 2);
    auto a = lua::tovec<n, number_t>(L, 1);
    auto b = lua::tovec<n, number_t>(L, 2);
    return lua::pushnumber(L, glm::dot(a, b));
}

template <int n>
static int l_inverse(lua::State* L) {
    uint argc = lua::check_argc(L, 1, 2);
    auto vec = lua::tovec<n, number_t>(L, 1);
    switch (argc) {
        case 1:
            lua::createtable(L, n, 0);
            for (uint i = 0; i < n; i++) {
                lua::pushnumber(L, (-1) * vec[i]);
                lua::rawseti(L, i + 1);
            }
            return 1;
        case 2:
            return lua::setvec(L, 2, -vec);
    }
    return 0;
}

static int l_spherical_rand(lua::State* L) {
    uint argc = lua::check_argc(L, 1, 2);
    switch (argc) {
        case 1:
            return lua::pushvec3(L, glm::sphericalRand(lua::tonumber(L, 1)));
        case 2:
            return lua::setvec(
                L,
                2,
                glm::sphericalRand(lua::tonumber(L, 1))
            );
    }
    return 0;
}

static int l_vec2_angle(lua::State* L) {
    uint argc = lua::check_argc(L, 1, 2);
    if (argc == 1) {
        return lua::pushnumber(L, glm::degrees(angle(lua::tovec2(L, 1))));
    } else {
        return lua::pushnumber(
            L,
            glm::degrees(
                angle(glm::vec2(lua::tonumber(L, 1), lua::tonumber(L, 2)))
            )
        );
    }
}

static int l_vec2_rotate(lua::State* L) {
    uint argc = lua::check_argc(L, 2, 3);
    auto vec = lua::tovec<2, number_t>(L, 1);
    auto angle = glm::radians(lua::tonumber(L, 2));

    if (argc == 2) {
        return lua::pushvec(L, glm::rotate(vec, angle));
    } else {
        return lua::setvec(L, 3, glm::rotate(vec, angle));
    }
}

template <int n>
static int l_tostring(lua::State* L) {
    lua::check_argc(L, 1);
    auto vec = lua::tovec<n, number_t>(L, 1);
    std::stringstream ss;
    ss << "vec" << std::to_string(n) << "{";
    for (int i = 0; i < n; i++) {
        if (i > 0) {
            ss << ", ";
        }
        ss << vec[i];
    }
    ss << "}";
    return lua::pushstring(L, ss.str());
}

#define VECLIB_BASE(dimension)                                  \
{"add", lua::wrap<l_binop<dimension, std::plus>>},              \
{"sub", lua::wrap<l_binop<dimension, std::minus>>},             \
{"mul", lua::wrap<l_binop<dimension, std::multiplies>>},        \
{"div", lua::wrap<l_binop<dimension, std::divides>>},           \
{"distance", lua::wrap<l_distance<dimension>>},                 \
{"normalize", lua::wrap<l_unaryop<dimension, glm::normalize>>}, \
{"length", lua::wrap<l_scalar_op<dimension, glm::length>>},     \
{"tostring", lua::wrap<l_tostring<dimension>>},                 \
{"abs", lua::wrap<l_unaryop<dimension, glm::abs>>},             \
{"round", lua::wrap<l_unaryop<dimension, glm::round>>},         \
{"inverse", lua::wrap<l_inverse<dimension>>},                   \
{"pow", lua::wrap<l_pow<dimension>>},                           \
{"dot", lua::wrap<l_dot<dimension>>},                           \
{"mix", lua::wrap<l_mix<dimension>>},

const luaL_Reg vec2lib[] = {
    VECLIB_BASE(2)
    {"angle", lua::wrap<l_vec2_angle>},
    {"rotate", lua::wrap<l_vec2_rotate>},
    {nullptr, nullptr}
};

const luaL_Reg vec3lib[] = {
    VECLIB_BASE(3)
    {"spherical_rand", lua::wrap<l_spherical_rand>},
    {nullptr, nullptr}
};

const luaL_Reg vec4lib[] = {
    VECLIB_BASE(4)
    {nullptr, nullptr}
};
