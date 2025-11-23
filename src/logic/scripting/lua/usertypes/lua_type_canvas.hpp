#pragma once

#include "../lua_commons.hpp"
#include "maths/UVRegion.hpp"
#include "constants.hpp"

class Texture;
class ImageData;

namespace lua {
    class LuaCanvas : public Userdata {
    public:
        explicit LuaCanvas(
            std::shared_ptr<Texture> texture,
            std::shared_ptr<ImageData> data,
            UVRegion region = UVRegion(0, 0, 1, 1)
        );
        ~LuaCanvas() override = default;

        const std::string& getTypeName() const override {
            return TYPENAME;
        }

        [[nodiscard]] auto& getTexture() const {
            return *texture;
        }

        [[nodiscard]] auto& getData() const {
            return *data;
        }

        [[nodiscard]] bool hasTexture() const {
            return texture != nullptr;
        }

        auto shareTexture() const {
            return texture;
        }

        void update(int extrusion = ATLAS_EXTRUSION);

        void createTexture();
        void unbindTexture();

        static int createMetatable(lua::State*);
        inline static std::string TYPENAME = "Canvas";
    private:
        std::shared_ptr<Texture> texture; // nullable
        std::shared_ptr<ImageData> data;
        UVRegion region;
    };
    static_assert(!std::is_abstract<LuaCanvas>());
}
