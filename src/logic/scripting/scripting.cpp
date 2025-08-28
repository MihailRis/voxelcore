#include "scripting.hpp"

#include <iostream>
#include <stdexcept>

#include "scripting_commons.hpp"
#include "content/Content.hpp"
#include "content/ContentPack.hpp"
#include "content/ContentControl.hpp"
#include "debug/Logger.hpp"
#include "engine/Engine.hpp"
#include "io/engine_paths.hpp"
#include "io/io.hpp"
#include "frontend/UiDocument.hpp"
#include "items/Inventory.hpp"
#include "items/ItemDef.hpp"
#include "logic/BlocksController.hpp"
#include "logic/LevelController.hpp"
#include "lua/lua_engine.hpp"
#include "lua/lua_custom_types.hpp"
#include "maths/Heightmap.hpp"
#include "objects/Entities.hpp"
#include "objects/EntityDef.hpp"
#include "objects/Player.hpp"
#include "util/stringutil.hpp"
#include "util/timeutil.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunk.hpp"
#include "world/Level.hpp"
#include "world/World.hpp"
#include "interfaces/Process.hpp"

using namespace scripting;

static debug::Logger logger("scripting");

static inline const std::string STDCOMP = "stdcomp";

std::ostream* scripting::output_stream = &std::cout;
std::ostream* scripting::error_stream = &std::cerr;
Engine* scripting::engine = nullptr;
Level* scripting::level = nullptr;
const Content* scripting::content = nullptr;
const ContentIndices* scripting::indices = nullptr;
ContentControl* scripting::content_control = nullptr;
BlocksController* scripting::blocks = nullptr;
LevelController* scripting::controller = nullptr;

void scripting::load_script(const io::path& name, bool throwable) {
    io::path file = io::path("res:scripts") / name;
    std::string src = io::read_string(file);
    auto L = lua::get_main_state();
    lua::loadbuffer(L, 0, src, "core:scripts/"+name.string());
    if (throwable) {
        lua::call(L, 0, 0);
    } else {
        lua::call_nothrow(L, 0, 0);
    }
}

int scripting::load_script(
    int env,
    const std::string& type,
    const io::path& file,
    const std::string& fileName
) {
    std::string src = io::read_string(file);
    logger.info() << "script (" << type << ") " << file.string();
    return lua::execute(lua::get_main_state(), env, src, fileName);
}

void scripting::initialize(Engine* engine) {
    scripting::engine = engine;
    scripting::content_control = &engine->getContentControl();
    lua::initialize(engine->getPaths(), engine->getCoreParameters());

    load_script(io::path("stdlib.lua"), true);
    load_script(io::path("classes.lua"), true);
}

class LuaCoroutine : public Process {
    lua::State* L;
    int id;
    bool alive = true;
public:
    LuaCoroutine(lua::State* L, int id) : L(L), id(id) {
    }

    bool isActive() const override {
        return alive;
    }
    
    void update() override {
        if (!alive || id == 0) {
            return;
        }
        if (lua::requireglobal(L, "__vc_resume_coroutine")) {
            lua::pushinteger(L, id);
            if (lua::call(L, 1)) {
                alive = lua::toboolean(L, -1);
                lua::pop(L);
            }
        }
    }

    void waitForEnd() override {
        while (isActive()) {
            update();
        }
    }
    
    void terminate() override {
        lua::requireglobal(L, "__vc_stop_coroutine");
        lua::pushinteger(L, id);
        lua::pop(L, lua::call(L, 1));
        id = 0;
    }
};

class LuaProjectScript : public IClientProjectScript {
public:
    LuaProjectScript(lua::State* L, scriptenv env) : L(L), env(std::move(env)) {}

    void onScreenChange(const std::string& name, bool show) override {
        if (!lua::pushenv(L, *env)) {
            return;
        }
        if (!lua::getfield(L, "on_" + name + (show ? "_setup" : "_clear"))) {
            lua::pop(L);
            return;
        }
        lua::call_nothrow(L, 0, 0);
        lua::pop(L);
    }
private:
    lua::State* L;
    scriptenv env;
};

std::unique_ptr<IClientProjectScript> scripting::load_client_project_script(
    const io::path& script
) {
    auto L = lua::get_main_state();
    auto source = io::read_string(script);
    auto env = create_environment(nullptr);
    lua::pushenv(L, *env);
    if (lua::getglobal(L, "__vc_app")) {
        lua::setfield(L, "app");
    }
    lua::pop(L);

    lua::loadbuffer(L, *env, source, script.name());
    lua::call(L, 0);
    return std::make_unique<LuaProjectScript>(L, std::move(env));
}

std::unique_ptr<Process> scripting::start_coroutine(const io::path& script) {
    auto L = lua::get_main_state();
    auto method = "__vc_start_coroutine";
    if (lua::getglobal(L, method)) {
        auto source = io::read_string(script);
        lua::loadbuffer(L, 0, source, script.name());
        if (lua::call(L, 1)) {
            int id = lua::tointeger(L, -1);
            lua::pop(L, 1);
            return std::make_unique<LuaCoroutine>(L, id);
        }
        lua::pop(L);
    }
    return nullptr;
}

[[nodiscard]] scriptenv scripting::get_root_environment() {
    return std::make_shared<int>(0);
}

[[nodiscard]] scriptenv scripting::create_pack_environment(
    const ContentPack& pack
) {
    auto L = lua::get_main_state();
    int id = lua::create_environment(L, 0);
    lua::pushenv(L, id);
    lua::pushvalue(L, -1);
    lua::setfield(L, "PACK_ENV");
    lua::pushstring(L, pack.id);
    lua::setfield(L, "PACK_ID");
    lua::pop(L);
    return std::shared_ptr<int>(new int(id), [=](int* id) { //-V508
        lua::remove_environment(L, *id);
        delete id;
    });
}

[[nodiscard]] scriptenv scripting::create_environment(
    const scriptenv& parent
) {
    auto L = lua::get_main_state();
    int id = lua::create_environment(L, (parent ? *parent : 0));
    lua::pushenv(L, id);
    lua::pushvalue(L, -1);
    lua::setfield(L, "CUR_ENV");

    lua::pop(L);
    return std::shared_ptr<int>(new int(id), [=](int* id) { //-V508
        lua::remove_environment(L, *id);
        delete id;
    });
}

[[nodiscard]] scriptenv scripting::create_doc_environment(
    const scriptenv& parent, const std::string& name
) {
    auto L = lua::get_main_state();
    int id = lua::create_environment(L, *parent);
    lua::pushenv(L, id);
    lua::pushvalue(L, -1);
    lua::setfield(L, "DOC_ENV");
    lua::pushstring(L, name);
    lua::setfield(L, "DOC_NAME");

    if (lua::getglobal(L, "Document")) {
        if (lua::getfield(L, "new")) {
            lua::pushstring(L, name);
            if (lua::call_nothrow(L, 1)) {
                lua::setfield(L, "document", -3);
            }
        }
        lua::pop(L);
    }
    lua::pop(L);
    return std::shared_ptr<int>(new int(id), [=](int* id) { //-V508
        lua::remove_environment(L, *id);
        delete id;
    });
}

[[nodiscard]] static scriptenv create_component_environment(
    const scriptenv& parent, int entityIdx, const std::string& name
) {
    auto L = lua::get_main_state();
    int id = lua::create_environment(L, *parent);

    lua::pushvalue(L, entityIdx);

    lua::pushenv(L, id);

    lua::pushvalue(L, -1);
    lua::setfield(L, "this");

    lua::pushvalue(L, -2);
    lua::setfield(L, "entity");

    lua::pop(L);
    if (lua::getfield(L, "components")) {
        lua::pushenv(L, id);
        lua::setfield(L, name);
        lua::pop(L);
    }
    lua::pop(L);

    return std::shared_ptr<int>(new int(id), [=](int* id) { //-V508
        lua::remove_environment(L, *id);
        delete id;
    });
}

void scripting::process_post_runnables() {
    auto L = lua::get_main_state();
    if (lua::getglobal(L, "__process_post_runnables")) {
        lua::call_nothrow(L, 0, 0);
    }
}

template <class T>
static int push_properties_tables(
    lua::State* L, const ContentUnitIndices<T>& indices
) {
    const auto units = indices.getDefs();
    size_t size = indices.count();
    lua::createtable(L, size, 0);
    for (size_t i = 0; i < size; i++) {
        lua::pushvalue(L, units[i]->properties);
        lua::rawseti(L, i);
    }
    return 1;
}

void scripting::on_content_load(Content* content) {
    scripting::content = content;
    scripting::indices = content->getIndices();

    const auto& indices = *content->getIndices();

    auto L = lua::get_main_state();
    if (lua::getglobal(L, "block")) {
        const auto& materials = content->getBlockMaterials();
        lua::createtable(L, 0, materials.size());
        for (const auto& [name, material] : materials) {
            lua::pushvalue(L, material->toTable());
            lua::setfield(L, name);
        }
        lua::setfield(L, "materials");
        
        push_properties_tables(L, indices.blocks);
        lua::setfield(L, "properties");
        lua::pop(L);
    }
    if (lua::getglobal(L, "item")) {
        push_properties_tables(L, indices.items);
        lua::setfield(L, "properties");
        lua::pop(L);
    }
    load_script("post_content.lua", true);
    load_script("stdcmd.lua", true);
}

void scripting::on_content_reset() {
    scripting::content = nullptr;
    scripting::indices = nullptr;
}

void scripting::on_world_load(LevelController* controller) {
    scripting::level = controller->getLevel();
    scripting::blocks = controller->getBlocksController();
    scripting::controller = controller;

    auto L = lua::get_main_state();
    if (lua::getglobal(L, "__vc_on_world_open")) {
        lua::call_nothrow(L, 0, 0);
    } 
    
    for (auto& pack : content_control->getAllContentPacks()) {
        lua::emit_event(L, pack.id + ":.worldopen", [](auto L) {
            return lua::pushboolean(
                L, !scripting::level->getWorld()->getInfo().isLoaded
            );
        });
    }
}

void scripting::on_world_tick(int tps) {
    auto L = lua::get_main_state();
    if (lua::getglobal(L, "__vc_on_world_tick")) {
        lua::pushinteger(L, tps);
        lua::call_nothrow(L, 1, 0);
    } 
    for (auto& pack : content_control->getAllContentPacks()) {
        lua::emit_event(L, pack.id + ":.worldtick");
    }
}

void scripting::on_world_save() {
    auto L = lua::get_main_state();
    for (auto& pack : content_control->getAllContentPacks()) {
        lua::emit_event(L, pack.id + ":.worldsave");
    }
    if (lua::getglobal(L, "__vc_on_world_save")) {
        lua::call_nothrow(L, 0, 0);
    }
}

void scripting::on_world_quit() {
    auto L = lua::get_main_state();
    for (auto& pack : content_control->getAllContentPacks()) {
        lua::emit_event(L, pack.id + ":.worldquit");
    }
    if (lua::getglobal(L, "__vc_on_world_quit")) {
        lua::call_nothrow(L, 0, 0);
    }
    scripting::level = nullptr;
    scripting::content = nullptr;
    scripting::indices = nullptr;
    scripting::blocks = nullptr;
    scripting::controller = nullptr;
}

void scripting::cleanup() {
    auto L = lua::get_main_state();
    lua::requireglobal(L, "pack");
    for (auto& pack : content_control->getAllContentPacks()) {
        lua::requirefield(L, "unload");
        lua::pushstring(L, pack.id);
        lua::call_nothrow(L, 1);
    }
    lua::pop(L);

    if (lua::getglobal(L, "__scripts_cleanup")) {
        lua::call_nothrow(L, 0);
    }
}

void scripting::on_blocks_tick(const Block& block, int tps) {
    std::string name = block.name + ".blockstick";
    lua::emit_event(lua::get_main_state(), name, [tps](auto L) {
        return lua::pushinteger(L, tps);
    });
}

void scripting::update_block(const Block& block, const glm::ivec3& pos) {
    std::string name = block.name + ".update";
    lua::emit_event(lua::get_main_state(), name, [pos](auto L) {
        return lua::pushivec_stack(L, pos);
    });
}

void scripting::random_update_block(const Block& block, const glm::ivec3& pos) {
    std::string name = block.name + ".randupdate";
    lua::emit_event(lua::get_main_state(), name, [pos](auto L) {
        return lua::pushivec_stack(L, pos);
    });
}

/// TODO: replace template with index
template<bool WorldFuncsSet::*worldfunc>
static bool on_block_common(
    const std::string& suffix,
    bool blockfunc,
    Player* player,
    const Block& block,
    const glm::ivec3& pos
) {
    bool result = false;
    if (blockfunc) {
        std::string name = block.name + "." + suffix;
        result =
            lua::emit_event(lua::get_main_state(), name, [pos, player](auto L) {
                lua::pushivec_stack(L, pos);
                lua::pushinteger(L, player ? player->getId() : -1);
                return 4;
            });
    }
    auto args = [&](lua::State* L) {
        lua::pushinteger(L, block.rt.id);
        lua::pushivec_stack(L, pos);
        lua::pushinteger(L, player ? player->getId() : -1);
        return 5;
    };
    for (auto& [packid, pack] : content->getPacks()) {
        if (pack->worldfuncsset.*worldfunc) {
            lua::emit_event(
                lua::get_main_state(), packid + ":.block" + suffix, args
            );
        }
    }
    return result;
}

void scripting::on_block_placed(
    Player* player, const Block& block, const glm::ivec3& pos
) {
    on_block_common<&WorldFuncsSet::onblockplaced>(
        "placed", block.rt.funcsset.onplaced, player, block, pos
    );
}

void scripting::on_block_replaced(
    Player* player, const Block& block, const glm::ivec3& pos
) {
    on_block_common<&WorldFuncsSet::onblockreplaced>(
        "replaced", block.rt.funcsset.onreplaced, player, block, pos
    );
}

void scripting::on_block_breaking(
    Player* player, const Block& block, const glm::ivec3& pos
) {
    on_block_common<&WorldFuncsSet::onblockbreaking>(
        "breaking", block.rt.funcsset.onbreaking, player, block, pos
    );
}

void scripting::on_block_broken(
    Player* player, const Block& block, const glm::ivec3& pos
) {
    on_block_common<&WorldFuncsSet::onblockbroken>(
        "broken", block.rt.funcsset.onbroken, player, block, pos
    );
}

bool scripting::on_block_interact(
    Player* player, const Block& block, const glm::ivec3& pos
) {
    return on_block_common<&WorldFuncsSet::onblockinteract>(
        "interact", block.rt.funcsset.oninteract, player, block, pos
    );
}

void scripting::on_chunk_present(const Chunk& chunk, bool loaded) {
    auto args = [&chunk, loaded](lua::State* L) {
        lua::pushvec_stack<2>(L, {chunk.x, chunk.z});
        lua::pushboolean(L, loaded);
        return 3;
    };
    for (auto& [packid, pack] : content->getPacks()) {
        if (pack->worldfuncsset.onchunkpresent) {
            lua::emit_event(
                lua::get_main_state(), packid + ":.chunkpresent", args
            );
        }
    }
}

void scripting::on_chunk_remove(const Chunk& chunk) {
    auto args = [&chunk](lua::State* L) {
        lua::pushvec_stack<2>(L, {chunk.x, chunk.z});
        return 2;
    };
    for (auto& [packid, pack] : content->getPacks()) {
        if (pack->worldfuncsset.onchunkremove) {
            lua::emit_event(
                lua::get_main_state(), packid + ":.chunkremove", args
            );
        }
    }
}

void scripting::on_inventory_open(const Player* player, const Inventory& inventory) {
    auto args = [player, &inventory](lua::State* L) {
        lua::pushinteger(L, inventory.getId());
        lua::pushinteger(L, player ? player->getId() : -1);
        return 2;
    };
    for (auto& [packid, pack] : content->getPacks()) {
        if (pack->worldfuncsset.oninventoryopen) {
            lua::emit_event(
                lua::get_main_state(), packid + ":.inventoryopen", args
            );
        }
    }
}

void scripting::on_inventory_closed(const Player* player, const Inventory& inventory) {
    auto args = [player, &inventory](lua::State* L) {
        lua::pushinteger(L, inventory.getId());
        lua::pushinteger(L, player ? player->getId() : -1);
        return 2;
    };
    for (auto& [packid, pack] : content->getPacks()) {
        if (pack->worldfuncsset.oninventoryclosed) {
            lua::emit_event(
                lua::get_main_state(), packid + ":.inventoryclosed", args
            );
        }
    }
}

void scripting::on_player_tick(Player* player, int tps) {
    auto args = [=](lua::State* L) {
        lua::pushinteger(L, player ? player->getId() : -1);
        lua::pushinteger(L, tps);
        return 2;
    };
    for (auto& [packid, pack] : content->getPacks()) {
        if (pack->worldfuncsset.onplayertick) {
            lua::emit_event(
                lua::get_main_state(), packid + ":.playertick", args
            );
        }
    }
}

bool scripting::on_item_use(Player* player, const ItemDef& item) {
    std::string name = item.name + ".use";
    return lua::emit_event(
        lua::get_main_state(),
        name,
        [player](lua::State* L) { return lua::pushinteger(L, player->getId()); }
    );
}

bool scripting::on_item_use_on_block(
    Player* player, const ItemDef& item, glm::ivec3 ipos, glm::ivec3 normal
) {
    std::string name = item.name + ".useon";
    return lua::emit_event(
        lua::get_main_state(),
        name,
        [ipos, normal, player](auto L) {
            lua::pushivec_stack(L, ipos);
            lua::pushinteger(L, player->getId());
            lua::pushivec(L, normal);
            return 5;
        }
    );
}

bool scripting::on_item_break_block(
    Player* player, const ItemDef& item, int x, int y, int z
) {
    std::string name = item.name + ".blockbreakby";
    return lua::emit_event(
        lua::get_main_state(),
        name,
        [x, y, z, player](auto L) {
            lua::pushivec_stack(L, glm::ivec3(x, y, z));
            lua::pushinteger(L, player->getId());
            return 4;
        }
    );
}

dv::value scripting::get_component_value(
    const scriptenv& env, const std::string& name
) {
    auto L = lua::get_main_state();
    lua::pushenv(L, *env);
    if (lua::getfield(L, name)) {
        return lua::tovalue(L, -1);
    }
    return nullptr;
}

void scripting::on_entity_spawn(
    const EntityDef&,
    entityid_t eid,
    const std::vector<std::unique_ptr<UserComponent>>& components,
    const dv::value& args,
    const dv::value& saved
) {
    auto L = lua::get_main_state();
    lua::stackguard guard(L);
    lua::requireglobal(L, STDCOMP);
    if (lua::getfield(L, "new_Entity")) {
        lua::pushinteger(L, eid);
        lua::call(L, 1);
    }
    if (components.size() > 1) {
        for (size_t i = 0; i < components.size() - 1; i++) {
            lua::pushvalue(L, -1);
        }
    }
    for (auto& component : components) {
        auto compenv = create_component_environment(
            get_root_environment(), -1, component->name
        );
        lua::get_from(L, lua::CHUNKS_TABLE, component->name, true);
        lua::pushenv(L, *compenv);

        if (args != nullptr) {
            std::string compfieldname = component->name;
            util::replaceAll(compfieldname, ":", "__");
            if (args.has(compfieldname)) {
                lua::pushvalue(L, args[compfieldname]);
            } else {
                lua::createtable(L, 0, 0);
            }
        } else {
            lua::createtable(L, 0, 0);
        }
        lua::setfield(L, "ARGS");

        if (saved == nullptr) {
            lua::createtable(L, 0, 0);
        } else {
            if (saved.has(component->name)) {
                lua::pushvalue(L, saved[component->name]);
            } else {
                lua::createtable(L, 0, 0);
            }
        }
        lua::setfield(L, "SAVED_DATA");

        lua::setfenv(L);
        lua::call_nothrow(L, 0, 0);

        lua::pushenv(L, *compenv);
        auto& funcsset = component->funcsset;
        funcsset.on_grounded = lua::hasfield(L, "on_grounded");
        funcsset.on_fall = lua::hasfield(L, "on_fall");
        funcsset.on_despawn = lua::hasfield(L, "on_despawn");
        funcsset.on_sensor_enter = lua::hasfield(L, "on_sensor_enter");
        funcsset.on_sensor_exit = lua::hasfield(L, "on_sensor_exit");
        funcsset.on_save = lua::hasfield(L, "on_save");
        funcsset.on_aim_on = lua::hasfield(L, "on_aim_on");
        funcsset.on_aim_off = lua::hasfield(L, "on_aim_off");
        funcsset.on_attacked = lua::hasfield(L, "on_attacked");
        funcsset.on_used = lua::hasfield(L, "on_used");
        lua::pop(L, 2);

        component->env = compenv;
    }
}

static void process_entity_callback(
    const scriptenv& env,
    const std::string& name,
    std::function<int(lua::State*)> args
) {
    auto L = lua::get_main_state();
    lua::pushenv(L, *env);
    if (lua::hasfield(L, "__disabled")) {
        lua::pop(L);
        return;
    }
    if (lua::getfield(L, name)) {
        if (args) {
            lua::call_nothrow(L, args(L), 0);
        } else {
            lua::call_nothrow(L, 0, 0);
        }
    }
    lua::pop(L);
}

static void process_entity_callback(
    const Entity& entity,
    const std::string& name,
    bool EntityFuncsSet::*flag,
    std::function<int(lua::State*)> args
) {
    const auto& script = entity.getScripting();
    for (auto& component : script.components) {
        if (component->funcsset.*flag) {
            process_entity_callback(component->env, name, args);
        }
    }
}

void scripting::on_entity_despawn(const Entity& entity) {
    process_entity_callback(
        entity, "on_despawn", &EntityFuncsSet::on_despawn, nullptr
    );
    auto L = lua::get_main_state();
    lua::get_from(L, "stdcomp", "remove_Entity", true);
    lua::pushinteger(L, entity.getUID());
    lua::call(L, 1, 0);
}

void scripting::on_entity_grounded(const Entity& entity, float force) {
    process_entity_callback(
        entity,
        "on_grounded",
        &EntityFuncsSet::on_grounded,
        [force](auto L) { return lua::pushnumber(L, force); }
    );
}

void scripting::on_entity_fall(const Entity& entity) {
    process_entity_callback(
        entity, "on_fall", &EntityFuncsSet::on_fall, nullptr
    );
}

void scripting::on_entity_save(const Entity& entity) {
    process_entity_callback(
        entity, "on_save", &EntityFuncsSet::on_save, nullptr
    );
}

void scripting::on_sensor_enter(
    const Entity& entity, size_t index, entityid_t oid
) {
    process_entity_callback(
        entity,
        "on_sensor_enter",
        &EntityFuncsSet::on_sensor_enter,
        [index, oid](auto L) {
            lua::pushinteger(L, index);
            lua::pushinteger(L, oid);
            return 2;
        }
    );
}

void scripting::on_sensor_exit(
    const Entity& entity, size_t index, entityid_t oid
) {
    process_entity_callback(
        entity,
        "on_sensor_exit",
        &EntityFuncsSet::on_sensor_exit,
        [index, oid](auto L) {
            lua::pushinteger(L, index);
            lua::pushinteger(L, oid);
            return 2;
        }
    );
}

void scripting::on_aim_on(const Entity& entity, Player* player) {
    process_entity_callback(
        entity,
        "on_aim_on",
        &EntityFuncsSet::on_aim_on,
        [player](auto L) { return lua::pushinteger(L, player->getId()); }
    );
}

void scripting::on_aim_off(const Entity& entity, Player* player) {
    process_entity_callback(
        entity,
        "on_aim_off",
        &EntityFuncsSet::on_aim_off,
        [player](auto L) { return lua::pushinteger(L, player->getId()); }
    );
}

void scripting::on_attacked(
    const Entity& entity, Player* player, entityid_t attacker
) {
    process_entity_callback(
        entity,
        "on_attacked",
        &EntityFuncsSet::on_attacked,
        [player, attacker](auto L) {
            lua::pushinteger(L, attacker);
            lua::pushinteger(L, player->getId());
            return 2;
        }
    );
}

void scripting::on_entity_used(const Entity& entity, Player* player) {
    process_entity_callback(
        entity,
        "on_used",
        &EntityFuncsSet::on_used,
        [player](auto L) { return lua::pushinteger(L, player->getId()); }
    );
}

void scripting::on_entities_update(int tps, int parts, int part) {
    auto L = lua::get_main_state();
    lua::get_from(L, STDCOMP, "update", true);
    lua::pushinteger(L, tps);
    lua::pushinteger(L, parts);
    lua::pushinteger(L, part);
    lua::call_nothrow(L, 3, 0);
    lua::pop(L);
}

void scripting::on_entities_render(float delta) {
    auto L = lua::get_main_state();
    lua::get_from(L, STDCOMP, "render", true);
    lua::pushnumber(L, delta);
    lua::call_nothrow(L, 1, 0);
    lua::pop(L);
}

void scripting::on_ui_open(
    UiDocument* layout, std::vector<dv::value> args
) {
    auto argsptr =
        std::make_shared<std::vector<dv::value>>(std::move(args));
    std::string name = layout->getId() + ".open";
    lua::emit_event(lua::get_main_state(), name, [=](auto L) {
        for (const auto& value : *argsptr) {
            lua::pushvalue(L, value);
        }
        return argsptr->size();
    });
}

void scripting::on_ui_progress(
    UiDocument* layout, int workDone, int workTotal
) {
    std::string name = layout->getId() + ".progress";
    lua::emit_event(lua::get_main_state(), name, [=](auto L) {
        lua::pushinteger(L, workDone);
        lua::pushinteger(L, workTotal);
        return 2;
    });
}

void scripting::on_ui_close(UiDocument* layout, Inventory* inventory) {
    std::string name = layout->getId() + ".close";
    lua::emit_event(lua::get_main_state(), name, [inventory](auto L) {
        return lua::pushinteger(L, inventory ? inventory->getId() : 0);
    });
}

bool scripting::register_event(
    int env, const std::string& name, const std::string& id
) {
    auto L = lua::get_main_state();
    if (lua::pushenv(L, env) == 0) {
        lua::pushglobals(L);
    }
    bool success = true;
    lua::getglobal(L, "events");
    lua::getfield(L, "reset");
    lua::pushstring(L, id);
    if (!lua::getfield(L, name, -4)) {
        success = false;
        lua::pushnil(L);
    }
    lua::call_nothrow(L, 2);
    lua::pop(L);

    // remove previous name
    lua::pushnil(L);
    lua::setfield(L, name);
    return success;
}

int scripting::get_values_on_stack() {
    return lua::gettop(lua::get_main_state());
}

void scripting::load_content_script(
    const scriptenv& senv,
    const std::string& prefix,
    const io::path& file,
    const std::string& fileName,
    BlockFuncsSet& funcsset
) {
    int env = *senv;
    lua::pop(lua::get_main_state(), load_script(env, "block", file, fileName));

    funcsset = {};
    funcsset.init = register_event(env, "init", prefix + ".init");
    funcsset.update = register_event(env, "on_update", prefix + ".update");
    funcsset.randupdate =
        register_event(env, "on_random_update", prefix + ".randupdate");
    funcsset.onbreaking =
        register_event(env, "on_breaking", prefix + ".breaking");
    funcsset.onbroken = register_event(env, "on_broken", prefix + ".broken");
    funcsset.onplaced = register_event(env, "on_placed", prefix + ".placed");
    funcsset.onreplaced =
        register_event(env, "on_replaced", prefix + ".replaced");
    funcsset.oninteract =
        register_event(env, "on_interact", prefix + ".interact");
    funcsset.onblockstick =
        register_event(env, "on_blocks_tick", prefix + ".blockstick");
}

void scripting::load_content_script(
    const scriptenv& senv,
    const std::string& prefix,
    const io::path& file,
    const std::string& fileName,
    ItemFuncsSet& funcsset
) {
    int env = *senv;
    lua::pop(lua::get_main_state(), load_script(env, "item", file, fileName));

    funcsset = {};
    funcsset.init = register_event(env, "init", prefix + ".init");
    funcsset.on_use = register_event(env, "on_use", prefix + ".use");
    funcsset.on_use_on_block =
        register_event(env, "on_use_on_block", prefix + ".useon");
    funcsset.on_block_break_by =
        register_event(env, "on_block_break_by", prefix + ".blockbreakby");
}

void scripting::load_entity_component(
    const std::string& name, const io::path& file, const std::string& fileName
) {
    auto L = lua::get_main_state();
    std::string src = io::read_string(file);
    logger.info() << "script (component) " << file.string();
    lua::loadbuffer(L, 0, src, fileName);
    lua::store_in(L, lua::CHUNKS_TABLE, name);
}

void scripting::load_world_script(
    const scriptenv& senv,
    const std::string& prefix,
    const io::path& file,
    const std::string& fileName,
    WorldFuncsSet& funcsset
) {
    int env = *senv;
    lua::pop(lua::get_main_state(), load_script(env, "world", file, fileName));

    funcsset = {};
    register_event(env, "init", prefix + ".init");
    register_event(env, "on_world_open", prefix + ":.worldopen");
    register_event(env, "on_world_tick", prefix + ":.worldtick");
    register_event(env, "on_world_save", prefix + ":.worldsave");
    register_event(env, "on_world_quit", prefix + ":.worldquit");
    funcsset.onblockplaced =
        register_event(env, "on_block_placed", prefix + ":.blockplaced");
    funcsset.onblockbreaking =
        register_event(env, "on_block_breaking", prefix + ":.blockbreaking");
    funcsset.onblockbroken =
        register_event(env, "on_block_broken", prefix + ":.blockbroken");
    funcsset.onblockreplaced =
        register_event(env, "on_block_replaced", prefix + ":.blockreplaced");
    funcsset.onblockinteract =
        register_event(env, "on_block_interact", prefix + ":.blockinteract");
    funcsset.onplayertick =
        register_event(env, "on_player_tick", prefix + ":.playertick");
    funcsset.onchunkpresent =
        register_event(env, "on_chunk_present", prefix + ":.chunkpresent");
    funcsset.onchunkremove =
        register_event(env, "on_chunk_remove", prefix + ":.chunkremove");
    funcsset.oninventoryopen =
        register_event(env, "on_inventory_open", prefix + ":.inventoryopen");
    funcsset.oninventoryclosed =
        register_event(env, "on_inventory_closed", prefix + ":.inventoryclosed");
}

void scripting::load_layout_script(
    const scriptenv& senv,
    const std::string& prefix,
    const io::path& file,
    const std::string& fileName,
    uidocscript& script
) {
    int env = *senv;

    lua::pop(lua::get_main_state(), load_script(env, "layout", file, fileName));
    script.onopen = register_event(env, "on_open", prefix + ".open");
    script.onprogress =
        register_event(env, "on_progress", prefix + ".progress");
    script.onclose = register_event(env, "on_close", prefix + ".close");
}

void scripting::close() {
    lua::finalize();
    content = nullptr;
    indices = nullptr;
    level = nullptr;
}
