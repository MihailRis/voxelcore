#include "api_lua.hpp"

#include <sstream>

using namespace scripting;

static int l_blank(lua::State*) {
    return 0;
}

static int l_capture_output(lua::State* L) {
    int argc = lua::gettop(L) - 1;
    if (!lua::isfunction(L, 1)) {
        throw std::runtime_error("function expected as argument 1");
    }
    for (int i = 0; i < argc; i++) {
        lua::pushvalue(L, i + 2);
    }
    lua::pushvalue(L, 1);

    auto prev_output = output_stream;
    auto prev_error = error_stream;

    std::stringstream captured_output;

    output_stream = &captured_output;
    error_stream = &captured_output;
    
    lua::call_nothrow(L, argc, 0);

    output_stream = prev_output;
    error_stream = prev_error;
    
    lua::pushstring(L, captured_output.str());
    return 1;
}

const luaL_Reg corelib[] = {
    {"blank", lua::wrap<l_blank>},
    {"capture_output", lua::wrap<l_capture_output>},
    {nullptr, nullptr}
};
