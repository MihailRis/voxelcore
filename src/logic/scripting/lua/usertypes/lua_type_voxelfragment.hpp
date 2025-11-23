#pragma once

#include <array>

#include "../lua_commons.hpp"

class VoxelFragment;

namespace lua {
    class LuaVoxelFragment : public Userdata {
        std::array<std::shared_ptr<VoxelFragment>, 4> fragmentVariants;
    public:
        LuaVoxelFragment(
            std::array<std::shared_ptr<VoxelFragment>, 4> fragmentVariants
        );

        virtual ~LuaVoxelFragment();

        std::shared_ptr<VoxelFragment> getFragment(size_t rotation) const {
            return fragmentVariants.at(rotation & 0b11);
        }

        const std::string& getTypeName() const override {
            return TYPENAME;
        }

        static int createMetatable(lua::State*);
        inline static std::string TYPENAME = "VoxelFragment";
    };
    static_assert(!std::is_abstract<LuaVoxelFragment>());
}
