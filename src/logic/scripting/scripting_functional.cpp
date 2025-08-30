#include "scripting_functional.hpp"

#include "coders/json.hpp"
#include "debug/Logger.hpp"
#include "util/stringutil.hpp"
#include "util/type_helpers.hpp"
#include "lua/lua_engine.hpp"

using namespace scripting;

static debug::Logger logger("scripting_func");

runnable scripting::create_runnable(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    auto L = lua::get_main_state();
    try {
        lua::loadbuffer(L, *env, src, file);
        return lua::create_runnable(L);
    } catch (const lua::luaerror& err) {
        logger.error() << err.what();
        return []() {};
    }
}

static lua::State* process_callback(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    auto L = lua::get_main_state();
    try {
        if (lua::eval(L, *env, src, file) != 0) {
            return L;
        }
    } catch (lua::luaerror& err) {
        logger.error() << err.what();
    }
    return nullptr;
}

key_handler scripting::create_key_handler(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    return [=](int code) {
        if (auto L = process_callback(env, src, file)) {
            int top = lua::gettop(L);
            if (lua::isfunction(L, -1)) {
                lua::pushinteger(L, code);
                lua::call_nothrow(L, 1);
            }
            int returned = lua::gettop(L) - top + 1;
            if (returned) {
                bool x = lua::toboolean(L, -1);
                lua::pop(L, returned);
                return x;
            }
            return false;
        }
        return false;
    };
}

template<typename T, int(pushfunc)(lua::State*, remove_const_ref_if_primitive_t<const T&>)>
std::function<void(const T&)> create_consumer(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    return [=](const T& x) {
        if (auto L = process_callback(env, src, file)) {
            pushfunc(L, x);
            lua::call_nothrow(L, 1);
        }
    };
}

wstringconsumer scripting::create_wstring_consumer(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    return create_consumer<std::wstring, lua::pushwstring>(env, src, file);
}

stringconsumer scripting::create_string_consumer(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    return create_consumer<std::string, lua::pushstring>(env, src, file);
}

boolconsumer scripting::create_bool_consumer(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    return create_consumer<bool, lua::pushboolean>(env, src, file);
}

doubleconsumer scripting::create_number_consumer(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    return create_consumer<number_t, lua::pushnumber>(env, src, file);
}

template <typename T, T(tovalueFunc)(lua::State*, int)>
std::function<T()> create_supplier(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    return [=]() {
        if (auto L = process_callback(env, src, file)) {
            if (lua::isfunction(L, -1)) {
                lua::call_nothrow(L, 0);
            }
            auto str = tovalueFunc(L, -1);
            lua::pop(L);
            return str;
        }
        return T {};
    };
}

wstringsupplier scripting::create_wstring_supplier(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    return create_supplier<std::wstring, lua::require_wstring>(env, src, file);
}

boolsupplier scripting::create_bool_supplier(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    return create_supplier<bool, lua::toboolean>(env, src, file);
}

doublesupplier scripting::create_number_supplier(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    return create_supplier<number_t, lua::tonumber>(env, src, file);
}

wstringchecker scripting::create_wstring_validator(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    return [=](const std::wstring& x) {
        if (auto L = process_callback(env, src, file)) {
            lua::pushwstring(L, x);
            if (lua::call_nothrow(L, 1)) return lua::toboolean(L, -1);
        }
        return false;
    };
}

int_array_consumer scripting::create_int_array_consumer(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    return [=](const int arr[], size_t len) {
        if (auto L = process_callback(env, src, file)) {
            for (uint i = 0; i < len; i++) {
                lua::pushinteger(L, arr[i]);
            }
            lua::call_nothrow(L, len);
        }
    };
}

vec2supplier scripting::create_vec2_supplier(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    return [=]() {
        if (auto L = process_callback(env, src, file)) {
            if (lua::isfunction(L, -1)) {
                lua::call_nothrow(L, 0);
            }
            auto y = lua::tonumber(L, -1);
            lua::pop(L);
            auto x = lua::tonumber(L, -1);
            lua::pop(L);
            return glm::vec2(x, y);
        }
        return glm::vec2(0, 0);
    };
}

value_to_string_func scripting::create_tostring(
    const scriptenv& env, const std::string& src, const std::string& file
) {
    auto L = lua::get_main_state();
    try {
        lua::loadbuffer(L, *env, src, file);
        lua::call(L, 0, 1);
        auto func = lua::create_lambda(L);
        return [func](const dv::value& value) {
            auto result = func({value});
            return json::stringify(result, true, "  ");
        };
    } catch (const lua::luaerror& err) {
        logger.error() << err.what();
        return [](const auto& value) {
            return json::stringify(value, true, "  ");
        };
    }
}
