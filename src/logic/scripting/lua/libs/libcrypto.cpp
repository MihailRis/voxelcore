#include "api_lua.hpp"

static int l_encrypt(lua::State* L) {

}

const luaL_Reg cryptolib[] = {
    {"encrypt", lua::wrap<l_encrypt>},
    {NULL, NULL}
};
