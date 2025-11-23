#include "scripting.hpp"

#include <algorithm>
#include <functional>

#include "scripting_commons.hpp"
#include "typedefs.hpp"
#include "lua/lua_engine.hpp"
#include "lua/usertypes/lua_type_heightmap.hpp"
#include "content/Content.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunk.hpp"
#include "data/dv.hpp"
#include "world/generator/GeneratorDef.hpp"
#include "util/timeutil.hpp"
#include "io/io.hpp"
#include "engine/Engine.hpp"
#include "debug/Logger.hpp"

using namespace lua;

static debug::Logger logger("generator-scripting");

class LuaGeneratorScript : public GeneratorScript {
    State* L;
    const GeneratorDef& def;
    scriptenv env = nullptr;

    io::path file;
    std::string dirPath;
public:
    LuaGeneratorScript(
        State* L,
        const GeneratorDef& def,
        const io::path& file,
        const std::string& dirPath
    )
        : L(L), def(def), file(file), dirPath(dirPath) {
    }

    virtual ~LuaGeneratorScript() {
        env.reset();
        if (L != get_main_state()) {
            close(L);
        }
    }

    void initialize(uint64_t seed) override {
        env = create_environment(L);
        stackguard _(L);

        pushenv(L, *env);
        pushstring(L, dirPath);
        setfield(L, "__DIR__");
        pushstring(L, dirPath + "/script.lua");
        setfield(L, "__FILE__");
        pushinteger(L, seed);
        setfield(L, "SEED");

        pop(L);

        if (io::exists(file)) {
            std::string src = io::read_string(file);
            logger.info() << "script (generator) " << file.string();
            pop(L, execute(L, *env, src, file.string()));
        } else {
            // Use default (empty) script
            pop(L, execute(L, *env, "", "<empty>"));
        }
    }

    std::shared_ptr<Heightmap> generateHeightmap(
        const glm::ivec2& offset,
        const glm::ivec2& size,
        uint bpd,
        const std::vector<std::shared_ptr<Heightmap>>& inputs
    ) override {
        pushenv(L, *env);
        if (getfield(L, "generate_heightmap")) {
            pushivec_stack(L, offset);
            pushivec_stack(L, size);
            pushinteger(L, bpd);
            if (!inputs.empty()) {
                size_t inputsNum = def.heightmapInputs.size();
                createtable(L, inputsNum, 0);
                for (size_t i = 0; i < inputsNum; i++) {
                    newuserdata<LuaHeightmap>(L, inputs[i]);
                    rawseti(L, i+1);
                }
            }
            if (call_nothrow(L, 5 + (!inputs.empty()))) {
                auto map = touserdata<LuaHeightmap>(L, -1)->getHeightmap();
                pop(L, 2);
                return map;
            }
        }
        pop(L);
        return std::make_shared<Heightmap>(size.x, size.y);
    }

    std::vector<std::shared_ptr<Heightmap>> generateParameterMaps(
        const glm::ivec2& offset, const glm::ivec2& size, uint bpd
    ) override {
        std::vector<std::shared_ptr<Heightmap>> maps;

        uint biomeParameters = def.biomeParameters;
        pushenv(L, *env);
        if (getfield(L, "generate_biome_parameters")) {
            pushivec_stack(L, offset);
            pushivec_stack(L, size);
            pushinteger(L, bpd);
            if (call_nothrow(L, 5, biomeParameters)) {
                for (int i = biomeParameters-1; i >= 0; i--) {
                    maps.push_back(
                        touserdata<LuaHeightmap>(L, -1-i)->getHeightmap());
                }
                pop(L, 1+biomeParameters);
                return maps;
            }
        }
        pop(L);
        for (uint i = 0; i < biomeParameters; i++) {
            maps.push_back(std::make_shared<Heightmap>(size.x, size.y));
        }
        return maps;
    }

    void perform_line(lua::State* L, std::vector<Placement>& placements) {
        rawgeti(L, 2);
        blockid_t block = touinteger(L, -1);
        pop(L);

        rawgeti(L, 3);
        glm::ivec3 a = tovec3(L, -1);
        pop(L);

        rawgeti(L, 4);
        glm::ivec3 b = tovec3(L, -1);
        pop(L);

        rawgeti(L, 5);
        int radius = touinteger(L, -1);
        pop(L);

        int priority = 0;
        if (objlen(L, -1) >= 6) {
            rawgeti(L, 6);
            priority = tointeger(L, -1);
            pop(L);
        }

        placements.emplace_back(priority, LinePlacement {block, a, b, radius});
    }

    void perform_placement(lua::State* L, std::vector<Placement>& placements) {
        rawgeti(L, 1);
        int structIndex = 0;
        if (isstring(L, -1)) {
            const char* name = require_string(L, -1);
            if (!std::strcmp(name, ":line")) {
                pop(L);

                perform_line(L, placements);
                return;
            }
            const auto& found = def.structuresIndices.find(name);
            if (found != def.structuresIndices.end()) {
                structIndex = found->second;
            }
        } else {
            structIndex = tointeger(L, -1);
        }
        pop(L);

        rawgeti(L, 2);
        glm::ivec3 pos = tovec3(L, -1);
        pop(L);

        rawgeti(L, 3);
        uint8_t rotation = tointeger(L, -1) & 0b11;
        pop(L);

        int priority = 1;
        if (objlen(L, -1) >= 4) {
            rawgeti(L, 4);
            priority = tointeger(L, -1);
            pop(L);
        }

        placements.emplace_back(
            priority, StructurePlacement {structIndex, pos, rotation}
        );
    }

    std::vector<Placement> placeStructuresWide(
        const glm::ivec2& offset, 
        const glm::ivec2& size, 
        uint chunkHeight
    ) override {
        std::vector<Placement> placements {};
        
        stackguard _(L);
        pushenv(L, *env);
        try {
            if (getfield(L, "place_structures_wide")) {
                pushivec_stack(L, offset);
                pushivec_stack(L, size);
                pushinteger(L, chunkHeight);
                if (call_nothrow(L, 5, 1)) {
                    int len = objlen(L, -1);
                    for (int i = 1; i <= len; i++) {
                        rawgeti(L, i);

                        perform_placement(L, placements);

                        pop(L);
                    }
                    pop(L);
                }
            }
        } catch (const std::runtime_error& err) {
            logger.error() << err.what();
        }
        return placements;
    }

    std::vector<Placement> placeStructures(
        const glm::ivec2& offset,
        const glm::ivec2& size,
        const std::shared_ptr<Heightmap>& heightmap,
        uint chunkHeight
    ) override {
        std::vector<Placement> placements {};
        
        stackguard _(L);
        pushenv(L, *env);
        if (getfield(L, "place_structures")) {
            pushivec_stack(L, offset);
            pushivec_stack(L, size);
            newuserdata<LuaHeightmap>(L, heightmap);
            pushinteger(L, chunkHeight);
            if (call_nothrow(L, 6, 1)) {
                int len = objlen(L, -1);
                for (int i = 1; i <= len; i++) {
                    rawgeti(L, i);

                    perform_placement(L, placements);

                    pop(L);
                }
                pop(L);
            }
        }
        return placements;
    }
};

std::unique_ptr<GeneratorScript> scripting::load_generator(
    const GeneratorDef& def,
    const io::path& file,
    const std::string& dirPath
) {
    auto L = create_state(Engine::getInstance().getPaths(), StateType::GENERATOR);

    return std::make_unique<LuaGeneratorScript>(L, def, file, dirPath);
}
