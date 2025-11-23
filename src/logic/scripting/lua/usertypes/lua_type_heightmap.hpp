#pragma once

#include "../lua_commons.hpp"

struct fnl_state;
class Heightmap;

namespace lua {
    class LuaHeightmap : public Userdata {
        std::shared_ptr<Heightmap> map;
        std::unique_ptr<fnl_state> noise;
    public:
        LuaHeightmap(const std::shared_ptr<Heightmap>& map);
        LuaHeightmap(uint width, uint height);

        virtual ~LuaHeightmap();

        uint getWidth() const;

        uint getHeight() const;

        float* getValues();

        const float* getValues() const;

        const std::string& getTypeName() const override {
            return TYPENAME;
        }

        const std::shared_ptr<Heightmap>& getHeightmap() const {
            return map;
        }

        fnl_state* getNoise() {
            return noise.get();
        }

        void setSeed(int64_t seed);

        static int createMetatable(lua::State*);
        inline static std::string TYPENAME = "Heightmap";
    };
    static_assert(!std::is_abstract<LuaHeightmap>());
}
