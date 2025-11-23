#pragma once

#include "../lua_commons.hpp"

namespace audio {
    class MemoryPCMStream;
}

namespace lua {
    class LuaPCMStream : public Userdata {
    public:
        explicit LuaPCMStream(std::shared_ptr<audio::MemoryPCMStream>&& stream);
        virtual ~LuaPCMStream() override;

        const std::shared_ptr<audio::MemoryPCMStream>& getStream() const;

        const std::string& getTypeName() const override {
            return TYPENAME;
        }
        static int createMetatable(lua::State*);
        inline static std::string TYPENAME = "__vc_PCMStream";
    private:
        std::shared_ptr<audio::MemoryPCMStream> stream;
    };
    static_assert(!std::is_abstract<LuaPCMStream>());
}
