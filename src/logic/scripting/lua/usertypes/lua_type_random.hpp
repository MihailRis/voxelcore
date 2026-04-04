#pragma once

#include "../lua_commons.hpp"

#include <random>

namespace lua {
    class LuaRandom : public Userdata {
    public:
        std::mt19937 rng;
        
        explicit LuaRandom(uint64_t seed) : rng(seed) {}
        virtual ~LuaRandom() override = default;

        const std::string& getTypeName() const override {
            return TYPENAME;
        }

        static int createMetatable(lua::State*);
        inline static std::string TYPENAME = "__vc_Random";
    };
    static_assert(!std::is_abstract<LuaRandom>());
}
