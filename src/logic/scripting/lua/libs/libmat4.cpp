#include <sstream>

#include "api_lua.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/ext/matrix_transform.hpp>
#include <glm/gtx/matrix_decompose.hpp>
#include <glm/gtx/quaternion.hpp>

/// Overloads:
/// mat4.idt() -> float[16] - creates identity matrix
/// mat4.idt(dst: float[16]) -> float[16] - sets dst to identity matrix
static int l_idt(lua::State* L) {
    uint argc = lua::check_argc(L, 0, 1);
    switch (argc) {
        case 0: {
            return lua::pushmat4(L, glm::mat4(1.0f));
        }
        case 1: {
            return lua::setmat4(L, 1, glm::mat4(1.0f));
        }
    }
    return 0;
}

/// mat4.determinant(matrix: float[16]) - calculates matrix determinant
static int l_determinant(lua::State* L) {
    if (lua::gettop(L) != 1) {
        throw std::runtime_error("invalid arguments number (1 expected)");
    }
    return lua::pushnumber(L, glm::determinant(lua::tomat4(L, 1)));
}

/// Overloads:
/// mat4.mul(m1: float[16], m2: float[16]) -> float[16] - creates matrix of m1
/// and m2 multiplication result mat4.mul(m1: float[16], m2: float[16], dst:
/// float[16]) -> float[16] - updates dst matrix with m1 and m2 multiplication
/// result mat4.mul(m1: float[16], v: float[3 or 4]) -> float[3 or 4] - creates
/// vector of m1 and v multiplication result mat4.mul(m1: float[16], v: float[3
/// or 4], dst: float[3 or 4]) -> float[3 or 4] - updates dst vector with m1 and
/// v multiplication result
static int l_mul(lua::State* L) {
    uint argc = lua::check_argc(L, 2, 3);
    auto matrix1 = lua::tomat4(L, 1);
    uint len2 = lua::objlen(L, 2);
    if (len2 < 3) {
        throw std::runtime_error("argument #2: vec3 or vec4 expected");
    }
    switch (argc) {
        case 2: {
            if (len2 == 4) {
                return lua::pushvec(L, matrix1 * lua::tovec4(L, 2));
            } else if (len2 == 3) {
                return lua::pushvec(
                    L, matrix1 * glm::vec4(lua::tovec3(L, 2), 1.0f)
                );
            }
            return lua::pushmat4(L, matrix1 * lua::tomat4(L, 2));
        }
        case 3: {
            if (len2 == 4) {
                return lua::setvec(L, 3, matrix1 * lua::tovec4(L, 2));
            } else if (len2 == 3) {
                return lua::setvec(
                    L, 3, matrix1 * glm::vec4(lua::tovec3(L, 2), 1.0f)
                );
            }
            return lua::setmat4(L, 3, matrix1 * lua::tomat4(L, 2));
        }
    }
    return 0;
}

/// Overloads:
/// mat4.<func>(vec: float[3]) -> float[16] - creates transform matrix
/// mat4.<func>(matrix: float[16], vec: float[3]) -> float[16] - creates
/// transformed copy of matrix mat4.<func>(matrix: float[16], vec: float[3],
/// dst: float[16]) -> sets dst to transformed version of matrix
template <glm::mat4 (*func)(const glm::mat4&, const glm::vec3&)>
inline int l_binop_func(lua::State* L) {
    uint argc = lua::gettop(L);
    switch (argc) {
        case 1: {
            auto vec = lua::tovec3(L, 1);
            return lua::pushmat4(L, func(glm::mat4(1.0f), vec));
        }
        case 2: {
            auto matrix = lua::tomat4(L, 1);
            auto vec = lua::tovec3(L, 2);
            return lua::pushmat4(L, func(matrix, vec));
        }
        case 3: {
            auto matrix = lua::tomat4(L, 1);
            auto vec = lua::tovec3(L, 2);
            return lua::setmat4(L, 3, func(matrix, vec));
        }
        default: {
            throw std::runtime_error(
                "invalid arguments number (1, 2 or 3 expected)"
            );
        }
    }
    return 0;
}

/// Overloads:
/// mat4.rotate(vec: float[3], angle: float) -> float[16] - creates rotation
/// matrix mat4.rotate(matrix: float[16], vec: float[3], angle: float) ->
/// float[16] - creates rotated copy of matrix mat4.rotate(matrix: float[16],
/// vec: float[3], angle: float, dst: float[16]) -> sets dst to rotated version
/// of matrix
inline int l_rotate(lua::State* L) {
    uint argc = lua::gettop(L);
    switch (argc) {
        case 2: {
            auto vec = lua::tovec3(L, 1);
            auto angle = glm::radians(static_cast<float>(lua::tonumber(L, 2)));
            return lua::pushmat4(L, glm::rotate(glm::mat4(1.0f), angle, vec));
        }
        case 3: {
            auto matrix = lua::tomat4(L, 1);
            auto vec = lua::tovec3(L, 2);
            auto angle = glm::radians(static_cast<float>(lua::tonumber(L, 3)));
            return lua::pushmat4(L, glm::rotate(matrix, angle, vec));
        }
        case 4: {
            auto matrix = lua::tomat4(L, 1);
            auto vec = lua::tovec3(L, 2);
            auto angle = glm::radians(static_cast<float>(lua::tonumber(L, 3)));
            return lua::setmat4(L, 4, glm::rotate(matrix, angle, vec));
        }
        default: {
            throw std::runtime_error(
                "invalid arguments number (2, 3 or 4 expected)"
            );
        }
    }
    return 0;
}

/// Overloads:
/// mat4.inverse(matrix: float[16]) -> float[16] - creates inversed version of
/// the matrix mat4.inverse(matrix: float[16], dst: float[16]) -> float[16] -
/// updates dst matrix with inversed version of the matrix
static int l_inverse(lua::State* L) {
    uint argc = lua::check_argc(L, 1, 2);
    auto matrix = lua::tomat4(L, 1);
    switch (argc) {
        case 1: {
            return lua::pushmat4(L, glm::inverse(matrix));
        }
        case 2: {
            return lua::setmat4(L, 2, glm::inverse(matrix));
        }
    }
    return 0;
}

/// Overloads:
/// mat4.transpose(matrix: float[16]) -> float[16] - creates transposed version
/// of the matrix mat4.transpose(matrix: float[16], dst: float[16]) -> float[16]
/// - updates dst matrix with transposed version of the matrix
static int l_transpose(lua::State* L) {
    uint argc = lua::check_argc(L, 1, 2);
    auto matrix = lua::tomat4(L, 1);
    switch (argc) {
        case 1: {
            return lua::pushmat4(L, glm::transpose(matrix));
        }
        case 2: {
            return lua::setmat4(L, 2, glm::transpose(matrix));
        }
    }
    return 0;
}

/// mat4.decompose(m: float[16]) -> {
///   scale=float[3],
///   rotation=float[16],
///   quaternion=float[4],
///   translation=float[3],
///   skew=float[3],
///   perspective=float[4]
/// } or nil
static int l_decompose(lua::State* L) {
    auto matrix = lua::tomat4(L, 1);
    glm::vec3 scale;
    glm::quat rotation;
    glm::vec3 translation;
    glm::vec3 skew;
    glm::vec4 perspective;
    if (glm::decompose(
            matrix, scale, rotation, translation, skew, perspective
        )) {
        lua::createtable(L, 0, 6);

        lua::pushvec3(L, scale);
        lua::setfield(L, "scale");

        lua::pushmat4(L, glm::toMat4(rotation));
        lua::setfield(L, "rotation");

        lua::pushquat(L, rotation);
        lua::setfield(L, "quaternion");

        lua::pushvec3(L, translation);
        lua::setfield(L, "translation");

        lua::pushvec3(L, skew);
        lua::setfield(L, "skew");

        lua::pushvec4(L, perspective);
        lua::setfield(L, "perspective");
        return 1;
    }
    return 0;
}

static int l_look_at(lua::State* L) {
    uint argc = lua::check_argc(L, 3, 4);
    auto eye = lua::tovec<3>(L, 1);
    auto center = lua::tovec<3>(L, 2);
    auto up = lua::tovec<3>(L, 3);

    if (argc == 3) {
        return lua::pushmat4(L, glm::lookAt(eye, center, up));
    } else {
        return lua::setmat4(L, 4, glm::lookAt(eye, center, up));
    }
}

static int l_from_quat(lua::State* L) {
    uint argc = lua::check_argc(L, 1, 2);
    auto quat = lua::toquat(L, 1);
    switch (argc) {
        case 1:
            return lua::pushmat4(L, glm::toMat4(quat));
        case 2:
            return lua::setmat4(L, 2, glm::toMat4(quat));
    }
    return 0;
}

static int l_tostring(lua::State* L) {
    auto matrix = lua::tomat4(L, 1);
    bool multiline = lua::toboolean(L, 2);
    std::stringstream ss;
    ss << "mat4 {";
    if (multiline) {
        ss << "\n";
    }
    for (uint y = 0; y < 4; y++) {
        for (uint x = 0; x < 4; x++) {
            if (multiline) {
                ss << "\t" << matrix[y][x];
            } else if (x > 0) {
                ss << " " << matrix[y][x];
            } else {
                ss << matrix[y][x];
            }
        }
        if (multiline) {
            ss << "\n";
        } else {
            ss << "; ";
        }
    }
    ss << "}";
    return lua::pushstring(L, ss.str());
}

const luaL_Reg mat4lib[] = {
    {"idt", lua::wrap<l_idt>},
    {"mul", lua::wrap<l_mul>},
    {"scale", lua::wrap<l_binop_func<glm::scale>>},
    {"rotate", lua::wrap<l_rotate>},
    {"translate", lua::wrap<l_binop_func<glm::translate>>},
    {"inverse", lua::wrap<l_inverse>},
    {"transpose", lua::wrap<l_transpose>},
    {"determinant", lua::wrap<l_determinant>},
    {"decompose", lua::wrap<l_decompose>},
    {"look_at", lua::wrap<l_look_at>},
    {"from_quat", lua::wrap<l_from_quat>},
    {"tostring", lua::wrap<l_tostring>},
    {nullptr, nullptr}
};
