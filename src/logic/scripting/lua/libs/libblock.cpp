#define VC_ENABLE_REFLECTION
#include "content/Content.hpp"
#include "content/ContentLoader.hpp"
#include "content/ContentControl.hpp"
#include "lighting/Lighting.hpp"
#include "logic/BlocksController.hpp"
#include "logic/LevelController.hpp"
#include "objects/Players.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/voxel.hpp"
#include "voxels/GlobalChunks.hpp"
#include "voxels/blocks_agent.hpp"
#include "world/Level.hpp"
#include "maths/voxmaths.hpp"
#include "data/StructLayout.hpp"
#include "engine/Engine.hpp"
#include "api_lua.hpp"

using namespace scripting;

static inline const Block* require_block(lua::State* L) {
    auto indices = content->getIndices();
    auto id = lua::tointeger(L, 1);
    return indices->blocks.get(id);
}

static inline int l_get_def(lua::State* L) {
    if (auto def = require_block(L)) {
        return lua::pushstring(L, def->name);
    }
    return 0;
}

static int l_material(lua::State* L) {
    if (auto def = require_block(L)) {
        return lua::pushstring(L, def->material);
    }
    return 0;
}

static int l_is_solid_at(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    return lua::pushboolean(
        L, blocks_agent::is_solid_at(*level->chunks, x, y, z)
    );
}

static int l_count(lua::State* L) {
    return lua::pushinteger(L, indices->blocks.count());
}

static int l_index(lua::State* L) {
    auto name = lua::require_string(L, 1);
    return lua::pushinteger(L, content->blocks.require(name).rt.id);
}

static int l_is_extended(lua::State* L) {
    if (auto def = require_block(L)) {
        return lua::pushboolean(L, def->rt.extended);
    }
    return 0;
}

static int l_get_size(lua::State* L) {
    if (auto def = require_block(L)) {
        return lua::pushivec_stack(L, glm::ivec3(def->size));
    }
    return 0;
}

static int l_is_segment(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    const auto& vox = blocks_agent::require(*level->chunks, x, y, z);
    return lua::pushboolean(L, vox.state.segment);
}

static int l_seek_origin(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    const auto& vox = blocks_agent::require(*level->chunks, x, y, z);
    auto& def = indices->blocks.require(vox.id);
    return lua::pushivec_stack(
        L, blocks_agent::seek_origin(*level->chunks, {x, y, z}, def, vox.state)
    );
}

static int l_set(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    auto id = lua::tointeger(L, 4);
    auto state = lua::tointeger(L, 5);
    bool noupdate = lua::toboolean(L, 6);
    if (static_cast<size_t>(id) >= indices->blocks.count()) {
        return 0;
    }
    if (!blocks_agent::set(*level->chunks, x, y, z, id, int2blockstate(state))) {
        return 0;
    }

    auto chunksController = controller->getChunksController();
    if (chunksController == nullptr) {
        return 1;
    }
    if (chunksController->lighting) {
        Lighting& lighting = *chunksController->lighting;
        lighting.onBlockSet(x, y, z, id);
    }
    if (!noupdate) {
        blocks->updateSides(x, y, z);
    }
    return 0;
}

static int l_get(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    auto vox = blocks_agent::get(*level->chunks, x, y, z);
    int id = vox == nullptr ? -1 : vox->id;
    return lua::pushinteger(L, id);
}

template<int n>
static int get_axis(lua::State* L, const Block& def, int rotation) {
    const CoordSystem& rot = def.rotations.variants[rotation];
    return lua::pushivec_stack(L, rot.axes[n]);
}

template<int n>
static int get_axis(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    if (lua::gettop(L) == 2) {
        const auto& def = level->content.getIndices()->blocks.require(x);
        return get_axis<n>(L, def, y);
    }
    auto z = lua::tointeger(L, 3);

    glm::ivec3 defAxis {};
    defAxis[n] = 1;

    auto vox = blocks_agent::get(*level->chunks, x, y, z);
    if (vox == nullptr) {
        return lua::pushivec_stack(L, defAxis);
    }
    const auto& def = level->content.getIndices()->blocks.require(vox->id);
    if (!def.rotatable) {
        return lua::pushivec_stack(L, defAxis);
    } else {
        return get_axis<n>(L, def, vox->state.rotation);
    }
}

static int l_get_x(lua::State* L) {
    return get_axis<0>(L);
}

static int l_get_y(lua::State* L) {
    return get_axis<1>(L);
}

static int l_get_z(lua::State* L) {
    return get_axis<2>(L);
}

static int l_get_rotation(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    auto vox = blocks_agent::get(*level->chunks, x, y, z);
    int rotation = vox == nullptr ? 0 : vox->state.rotation;
    return lua::pushinteger(L, rotation);
}

static int l_set_rotation(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    auto value = lua::tointeger(L, 4);
    blocks_agent::set_rotation(*level->chunks, x, y, z, value);
    return 0;
}

static int l_get_states(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    auto vox = blocks_agent::get(*level->chunks, x, y, z);
    int states = vox == nullptr ? 0 : blockstate2int(vox->state);
    return lua::pushinteger(L, states);
}

static int l_set_states(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    auto states = lua::tointeger(L, 4);
    if (y < 0 || y >= CHUNK_H) {
        return 0;
    }
    int cx = floordiv<CHUNK_W>(x);
    int cz = floordiv<CHUNK_D>(z);
    auto chunk = blocks_agent::get_chunk(*level->chunks, cx, cz);
    if (chunk == nullptr) {
        return 0;
    }
    int lx = x - cx * CHUNK_W;
    int lz = z - cz * CHUNK_D;
    chunk->voxels[vox_index(lx, y, lz)].state = int2blockstate(states);
    chunk->setModifiedAndUnsaved();
    return 0;
}

static int l_get_user_bits(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);

    auto offset = lua::tointeger(L, 4) + VOXEL_USER_BITS_OFFSET;
    auto bits = lua::tointeger(L, 5);

    auto vox = blocks_agent::get(*level->chunks, x, y, z);
    if (vox == nullptr) {
        return lua::pushinteger(L, 0);
    }
    const auto& def = content->getIndices()->blocks.require(vox->id);
    if (def.rt.extended) {
        auto origin = blocks_agent::seek_origin(
            *level->chunks, {x, y, z}, def, vox->state
        );
        vox = blocks_agent::get(*level->chunks, origin.x, origin.y, origin.z);
        if (vox == nullptr) {
            return lua::pushinteger(L, 0);
        }
    }
    uint mask = ((1 << bits) - 1) << offset;
    return lua::pushinteger(L, (blockstate2int(vox->state) & mask) >> offset);
}

static int l_get_variant(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);

    auto vox = blocks_agent::get(*level->chunks, x, y, z);
    if (vox == nullptr) {
        return lua::pushinteger(L, 0);
    }
    const auto& def = content->getIndices()->blocks.require(vox->id);
    if (def.variants == nullptr) {
        return lua::pushinteger(L, 0);
    }
    return lua::pushinteger(
        L, (vox->state.userbits >> def.variants->offset) & def.variants->mask
    );
}

static int l_set_user_bits(lua::State* L) {
    auto& chunks = *level->chunks;
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    auto offset = lua::tointeger(L, 4);
    auto bits = lua::tointeger(L, 5);

    size_t mask = ((1 << bits) - 1) << offset;
    auto value = (lua::tointeger(L, 6) << offset) & mask;

    int cx = floordiv<CHUNK_W>(x);
    int cz = floordiv<CHUNK_D>(z);
    auto chunk = blocks_agent::get_chunk(chunks, cx, cz);
    if (chunk == nullptr || y < 0 || y >= CHUNK_H) {
        return 0;
    }
    int lx = x - cx * CHUNK_W;
    int lz = z - cz * CHUNK_D;
    auto vox = &chunk->voxels[vox_index(lx, y, lz)];
    const auto& def = content->getIndices()->blocks.require(vox->id);
    if (def.rt.extended) {
        auto origin = blocks_agent::seek_origin(chunks, {x, y, z}, def, vox->state);
        vox = blocks_agent::get(chunks, origin.x, origin.y, origin.z);
        if (vox == nullptr) {
            return 0;
        }
    }
    vox->state.userbits = (vox->state.userbits & (~mask)) | value;
    chunk->setModifiedAndUnsaved();
    return 0;
}

static int l_set_variant(lua::State* L) {
    auto& chunks = *level->chunks;
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);

    int cx = floordiv<CHUNK_W>(x);
    int cz = floordiv<CHUNK_D>(z);
    auto chunk = blocks_agent::get_chunk(chunks, cx, cz);
    if (chunk == nullptr || y < 0 || y >= CHUNK_H) {
        return 0;
    }
    int lx = x - cx * CHUNK_W;
    int lz = z - cz * CHUNK_D;
    auto vox = &chunk->voxels[vox_index(lx, y, lz)];
    const auto& def = content->getIndices()->blocks.require(vox->id);

    if (def.variants == nullptr) {
        return 0;
    }

    auto offset = def.variants->offset;
    auto mask = def.variants->mask;
    auto value = (lua::tointeger(L, 4) << offset) & mask;

    if (def.rt.extended) {
        auto origin = blocks_agent::seek_origin(chunks, {x, y, z}, def, vox->state);
        vox = blocks_agent::get(chunks, origin.x, origin.y, origin.z);
        if (vox == nullptr) {
            return 0;
        }
    }
    vox->state.userbits = (vox->state.userbits & (~mask)) | value;
    chunk->setModifiedAndUnsaved();
    return 0;
}

static int l_is_replaceable_at(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    return lua::pushboolean(
        L, blocks_agent::is_replaceable_at(*level->chunks, x, y, z)
    );
}

static int l_caption(lua::State* L) {
    if (auto def = require_block(L)) {
        return lua::pushstring(L, def->caption);
    }
    return 0;
}

static int l_get_textures(lua::State* L) {
    if (auto def = require_block(L)) {
        lua::createtable(L, 6, 0);
        for (size_t i = 0; i < 6; i++) {
            lua::pushstring(L, def->defaults.textureFaces[i]); // TODO: variant argument
            lua::rawseti(L, i + 1);
        }
        return 1;
    }
    return 0;
}


static int l_model_name(lua::State* L) {
    if (auto def = require_block(L)) {
        // TODO: variant argument
        const auto& modelName = def->defaults.model.name;
        if (modelName.empty()) {
            return lua::pushlstring(L, def->name + ".model");
        }
        return lua::pushlstring(L, modelName);
    }
    return 0;
}

static int l_get_model(lua::State* L) {
    if (auto def = require_block(L)) {
        // TODO: variant argument
        return lua::pushlstring(L, BlockModelTypeMeta.getName(def->defaults.model.type));
    }
    return 0;
}

static int l_get_hitbox(lua::State* L) {
    if (auto def = require_block(L)) {
        size_t rotation = lua::tointeger(L, 2);
        if (def->rotatable) {
            rotation %= def->rotations.MAX_COUNT;
        } else {
            rotation = 0;
        }
        auto& hitbox = def->rt.hitboxes[rotation].at(0);
        lua::createtable(L, 2, 0);

        lua::pushvec3(L, hitbox.min());
        lua::rawseti(L, 1);

        lua::pushvec3(L, hitbox.size());
        lua::rawseti(L, 2);
        return 1;
    }
    return 0;
}

static int l_get_rotation_profile(lua::State* L) {
    if (auto def = require_block(L)) {
        return lua::pushstring(L, def->rotations.name);
    }
    return 0;
}

static int l_get_picking_item(lua::State* L) {
    if (auto def = require_block(L)) {
        return lua::pushinteger(L, def->rt.pickingItem);
    }
    return 0;
}

static int l_place(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    auto id = lua::tointeger(L, 4);
    auto state = lua::tointeger(L, 5);
    auto playerid = lua::gettop(L) >= 6 ? lua::tointeger(L, 6) : -1;
    if (static_cast<size_t>(id) >= indices->blocks.count()) {
        return 0;
    }
    if (!blocks_agent::get(*level->chunks, x, y, z)) {
        return 0;
    }
    const auto def = level->content.getIndices()->blocks.get(id);
    if (def == nullptr) {
        throw std::runtime_error(
            "there is no block with index " + std::to_string(id)
        );
    }
    auto player = level->players->get(playerid);
    controller->getBlocksController()->placeBlock(
        player, *def, int2blockstate(state), x, y, z
    );
    return 0;
}

static int l_destruct(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    auto playerid = lua::gettop(L) >= 4 ? lua::tointeger(L, 4) : -1;
    auto vox = blocks_agent::get(*level->chunks, x, y, z);
    if (vox == nullptr) {
        return 0;
    }
    auto& def = level->content.getIndices()->blocks.require(vox->id);
    auto player = level->players->get(playerid);
    controller->getBlocksController()->breakBlock(player, def, x, y, z);
    return 0;
}

static int l_raycast(lua::State* L) {
    auto start = lua::tovec<3>(L, 1);
    auto dir = lua::tovec<3>(L, 2);
    auto maxDistance = lua::tonumber(L, 3);
    std::set<blockid_t> filteredBlocks {};
    if (lua::gettop(L) >= 5) {
        if (lua::istable(L, 5)) {
            int addLen = lua::objlen(L, 5);
            for (int i = 0; i < addLen; i++) {
                lua::rawgeti(L, i + 1, 5);
                auto blockName = std::string(lua::tostring(L, -1));
                const Block* block = content->blocks.find(blockName);
                if (block != nullptr) {
                    filteredBlocks.insert(block->rt.id);
                }
                lua::pop(L);
            }
        } else {
            throw std::runtime_error("table expected for filter");
        }
    }
    glm::vec3 end;
    glm::ivec3 normal;
    glm::ivec3 iend;
    if (auto voxel = blocks_agent::raycast(
            *level->chunks,
            start,
            dir,
            maxDistance,
            end,
            normal,
            iend,
            filteredBlocks
        )) {
        if (lua::gettop(L) >= 4 && !lua::isnil(L, 4)) {
            lua::pushvalue(L, 4);
        } else {
            lua::createtable(L, 0, 5);
        }

        lua::pushvec3(L, end);
        lua::setfield(L, "endpoint");

        lua::pushvec3(L, normal);
        lua::setfield(L, "normal");

        lua::pushnumber(L, glm::distance(start, end));
        lua::setfield(L, "length");

        lua::pushvec3(L, iend);
        lua::setfield(L, "iendpoint");

        lua::pushinteger(L, voxel->id);
        lua::setfield(L, "block");
        return 1;
    }
    return 0;
}

static int l_compose_state(lua::State* L) {
    if (!lua::istable(L, 1) || lua::objlen(L, 1) < 3) {
        throw std::runtime_error("expected array of 3 integers");
    }
    blockstate state {};

    lua::rawgeti(L, 1, 1);
    state.rotation = lua::tointeger(L, -1);
    lua::pop(L);
    lua::rawgeti(L, 2, 1);
    state.segment = lua::tointeger(L, -1);
    lua::pop(L);
    lua::rawgeti(L, 3, 1);
    state.userbits = lua::tointeger(L, -1);
    lua::pop(L);

    return lua::pushinteger(L, blockstate2int(state));
}

static int l_decompose_state(lua::State* L) {
    auto stateInt = static_cast<blockstate_t>(lua::tointeger(L, 1));
    auto state = int2blockstate(stateInt);

    lua::createtable(L, 3, 0);
    lua::pushinteger(L, state.rotation);
    lua::rawseti(L, 1);

    lua::pushinteger(L, state.segment);
    lua::rawseti(L, 2);

    lua::pushinteger(L, state.userbits);
    lua::rawseti(L, 3);
    return 1;
}

static int get_field(
    lua::State* L,
    const ubyte* src,
    const data::Field& field,
    size_t index,
    const data::StructLayout& dataStruct
) {
    switch (field.type) {
        case data::FieldType::I8:
        case data::FieldType::I16:
        case data::FieldType::I32:
        case data::FieldType::I64:
            return lua::pushinteger(L, dataStruct.getInteger(src, field, index));
        case data::FieldType::F32:
        case data::FieldType::F64:
            return lua::pushnumber(L, dataStruct.getNumber(src, field, index));
        case data::FieldType::CHAR:
            return lua::pushstring(L, 
                std::string(dataStruct.getChars(src, field)).c_str());
    }
    return 0;
}

static int l_get_field(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    auto name = lua::require_string(L, 4);
    size_t index = 0;
    if (lua::gettop(L) >= 5) {
        index = lua::tointeger(L, 5);
    }
    auto cx = floordiv(x, CHUNK_W);
    auto cz = floordiv(z, CHUNK_D);
    auto chunk = blocks_agent::get_chunk(*level->chunks, cx, cz);
    if (chunk == nullptr || y < 0 || y >= CHUNK_H) {
        return 0;
    }
    auto lx = x - cx * CHUNK_W;
    auto lz = z - cz * CHUNK_W;
    size_t voxelIndex = vox_index(lx, y, lz);

    const auto& vox = chunk->voxels[voxelIndex];
    const auto& def = content->getIndices()->blocks.require(vox.id);
    if (def.dataStruct == nullptr) {
        return 0;
    }
    const auto& dataStruct = *def.dataStruct;
    const auto field = dataStruct.getField(name);
    if (field == nullptr) {
        return 0;
    }
    if (index >= field->elements) {
        throw std::out_of_range(
            "index out of bounds [0, "+std::to_string(field->elements)+"]");
    }
    const ubyte* src = chunk->blocksMetadata.find(voxelIndex);
    if (src == nullptr) {
        return 0;
    }
    return get_field(L, src, *field, index, dataStruct);
}

static int set_field(
    lua::State* L,
    ubyte* dst,
    const data::Field& field,
    size_t index,
    const data::StructLayout& dataStruct,
    const dv::value& value
) {
    switch (field.type) {
        case data::FieldType::CHAR:
            if (value.isString()) {
                return lua::pushinteger(L,
                    dataStruct.setUnicode(dst, value.asString(), field));
            }
            [[fallthrough]];
        case data::FieldType::I8:
        case data::FieldType::I16:
        case data::FieldType::I32:
        case data::FieldType::I64:
            dataStruct.setInteger(dst, value.asInteger(), field, index);
            break;
        case data::FieldType::F32:
        case data::FieldType::F64:
            dataStruct.setNumber(dst, value.asNumber(), field, index);
            break;
    }
    return 0;
}

static int l_set_field(lua::State* L) {
    auto x = lua::tointeger(L, 1);
    auto y = lua::tointeger(L, 2);
    auto z = lua::tointeger(L, 3);
    auto name = lua::require_string(L, 4);
    auto value = lua::tovalue(L, 5);
    size_t index = 0;
    if (lua::gettop(L) >= 6) {
        index = lua::tointeger(L, 6);
    }
    auto cx = floordiv(x, CHUNK_W);
    auto cz = floordiv(z, CHUNK_D);
    auto lx = x - cx * CHUNK_W;
    auto lz = z - cz * CHUNK_W;
    auto chunk = blocks_agent::get_chunk(*level->chunks, cx, cz);
    if (chunk == nullptr || y < 0 || y >= CHUNK_H) {
        return 0;
    }
    size_t voxelIndex = vox_index(lx, y, lz);
    const auto& vox = chunk->voxels[voxelIndex];

    const auto& def = content->getIndices()->blocks.require(vox.id);
    if (def.dataStruct == nullptr) {
        return 0;
    }
    const auto& dataStruct = *def.dataStruct;
    const auto field = dataStruct.getField(name);
    if (field == nullptr) {
        return 0;
    }
    if (index >= field->elements) {
        throw std::out_of_range(
            "index out of bounds [0, "+std::to_string(field->elements)+"]");
    }
    ubyte* dst = chunk->blocksMetadata.find(voxelIndex);
    if (dst == nullptr) {
        dst = chunk->blocksMetadata.allocate(voxelIndex, dataStruct.size());
    }
    chunk->flags.unsaved = true;
    chunk->flags.blocksData = true;
    return set_field(L, dst, *field, index, dataStruct, value);
}

static int l_reload_script(lua::State* L) {
    auto name = lua::require_string(L, 1);
    if (content == nullptr) {
        throw std::runtime_error("content is not initialized");
    }
    auto& writeableContent = *content_control->get();
    auto& def = writeableContent.blocks.require(name);
    ContentLoader::reloadScript(writeableContent, def);
    return 0;
}

static int l_has_tag(lua::State* L) {
    if (auto def = require_block(L)) {
        auto tag = lua::require_string(L, 2);
        const auto& tags = def->rt.tags;
        return lua::pushboolean(L, tags.find(content->getTagIndex(tag)) != tags.end());
    }
    return 0;
}

static int l_pull_register_events(lua::State* L) {
    auto events = blocks_agent::pull_register_events();
    if (events.empty())
        return 0;

    lua::createtable(L, events.size() * 4, 0);
    for (int i = 0; i < events.size(); i++) {
        const auto& event = events[i];
        lua::pushinteger(L, static_cast<int>(event.type) | event.id << 16);
        lua::rawseti(L, i * 4 + 1);

        for (int j = 0; j < 3; j++) {
            lua::pushinteger(L, event.coord[j]);
            lua::rawseti(L, i * 4 + j + 2);
        }
    }
    return 1;
}

const luaL_Reg blocklib[] = {
    {"index", lua::wrap<l_index>},
    {"name", lua::wrap<l_get_def>},
    {"material", lua::wrap<l_material>},
    {"caption", lua::wrap<l_caption>},
    {"defs_count", lua::wrap<l_count>},
    {"is_solid_at", lua::wrap<l_is_solid_at>},
    {"is_replaceable_at", lua::wrap<l_is_replaceable_at>},
    {"set", lua::wrap<l_set>},
    {"get", lua::wrap<l_get>},
    {"get_X", lua::wrap<l_get_x>},
    {"get_Y", lua::wrap<l_get_y>},
    {"get_Z", lua::wrap<l_get_z>},
    {"get_states", lua::wrap<l_get_states>},
    {"set_states", lua::wrap<l_set_states>},
    {"get_rotation", lua::wrap<l_get_rotation>},
    {"set_rotation", lua::wrap<l_set_rotation>},
    {"get_user_bits", lua::wrap<l_get_user_bits>},
    {"set_user_bits", lua::wrap<l_set_user_bits>},
    {"get_variant", lua::wrap<l_get_variant>},
    {"set_variant", lua::wrap<l_set_variant>},
    {"is_extended", lua::wrap<l_is_extended>},
    {"get_size", lua::wrap<l_get_size>},
    {"is_segment", lua::wrap<l_is_segment>},
    {"seek_origin", lua::wrap<l_seek_origin>},
    {"model_name", lua::wrap<l_model_name>},
    {"get_textures", lua::wrap<l_get_textures>},
    {"get_model", lua::wrap<l_get_model>},
    {"get_hitbox", lua::wrap<l_get_hitbox>},
    {"get_rotation_profile", lua::wrap<l_get_rotation_profile>},
    {"get_picking_item", lua::wrap<l_get_picking_item>},
    {"place", lua::wrap<l_place>},
    {"destruct", lua::wrap<l_destruct>},
    {"raycast", lua::wrap<l_raycast>},
    {"compose_state", lua::wrap<l_compose_state>},
    {"decompose_state", lua::wrap<l_decompose_state>},
    {"get_field", lua::wrap<l_get_field>},
    {"set_field", lua::wrap<l_set_field>},
    {"reload_script", lua::wrap<l_reload_script>},
    {"has_tag", lua::wrap<l_has_tag>},
    {"__pull_register_events", lua::wrap<l_pull_register_events>},
    {NULL, NULL}
};
