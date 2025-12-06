#include "api_lua.hpp"

#include "frontend/hud.hpp"
#include "graphics/render/WorldRenderer.hpp"
#include "graphics/render/DebugLinesRenderer.hpp"
#include "graphics/render/ParticlesRenderer.hpp"
#include "graphics/render/ChunksRenderer.hpp"
#include "graphics/core/Mesh.hpp"
#include "logic/scripting/scripting.hpp"
#include "world/Level.hpp"
#include "objects/Entities.hpp"

using namespace scripting;

static int l_get_show_chunk_borders(lua::State* L) {
    return lua::pushboolean(L, WorldRenderer::showChunkBorders);
}

static int l_set_show_chunk_borders(lua::State* L) {
    WorldRenderer::showChunkBorders = lua::toboolean(L, 1);
    return 0;
}

static int l_get_show_hitboxes(lua::State* L) {
    return lua::pushboolean(L, WorldRenderer::showEntitiesDebug);
}

static int l_set_show_hitboxes(lua::State* L) {
    WorldRenderer::showEntitiesDebug = lua::toboolean(L, 1);
    return 0;
}

static int l_get_show_paths(lua::State* L) {
    return lua::pushboolean(L, DebugLinesRenderer::showPaths);
}

static int l_set_show_paths(lua::State* L) {
    DebugLinesRenderer::showPaths = lua::toboolean(L, 1);
    return 0;
}

static int l_get_show_generator_minimap(lua::State* L) {
    return lua::pushboolean(L, Hud::showGeneratorMinimap);
}

static int l_set_show_generator_minimap(lua::State* L) {
    Hud::showGeneratorMinimap = lua::toboolean(L, 1);
    return 0;
}

// --- Stats accessors ---
static int l_get_meshes_count(lua::State* L) {
    return lua::pushinteger(L, MeshStats::meshesCount);
}

static int l_get_draw_calls(lua::State* L) {
    int calls = MeshStats::drawCalls;
    MeshStats::drawCalls = 0;
    return lua::pushinteger(L, calls);
}

static int l_get_particles_visible(lua::State* L) {
    return lua::pushinteger(L, ParticlesRenderer::visibleParticles);
}

static int l_get_emitters_alive(lua::State* L) {
    return lua::pushinteger(L, ParticlesRenderer::aliveEmitters);
}

static int l_get_visible_chunks(lua::State* L) {
    return lua::pushinteger(L, ChunksRenderer::visibleChunks);
}

static int l_get_entities_next_id(lua::State* L) {
    if (scripting::level == nullptr || scripting::level->entities == nullptr) {
        return lua::pushinteger(L, 0);
    }
    return lua::pushinteger(L, scripting::level->entities->peekNextID());
}

const luaL_Reg renderlib[] = {
    {"get_show_chunk_borders", lua::wrap<l_get_show_chunk_borders>},
    {"set_show_chunk_borders", lua::wrap<l_set_show_chunk_borders>},
    {"get_show_hitboxes", lua::wrap<l_get_show_hitboxes>},
    {"set_show_hitboxes", lua::wrap<l_set_show_hitboxes>},
    {"get_show_paths", lua::wrap<l_get_show_paths>},
    {"set_show_paths", lua::wrap<l_set_show_paths>},
    {"get_show_generator_minimap", lua::wrap<l_get_show_generator_minimap>},
    {"set_show_generator_minimap", lua::wrap<l_set_show_generator_minimap>},
    // Stats
    {"get_meshes_count", lua::wrap<l_get_meshes_count>},
    {"get_draw_calls", lua::wrap<l_get_draw_calls>},
    {"get_particles_visible", lua::wrap<l_get_particles_visible>},
    {"get_emitters_alive", lua::wrap<l_get_emitters_alive>},
    {"get_visible_chunks", lua::wrap<l_get_visible_chunks>},
    {"get_entities_next_id", lua::wrap<l_get_entities_next_id>},
    {nullptr, nullptr}
};
