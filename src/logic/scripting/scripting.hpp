#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>

#include "io/fwd.hpp"
#include "data/dv.hpp"
#include "delegates.hpp"
#include "typedefs.hpp"
#include "scripting_functional.hpp"

class Engine;
class Content;
struct ContentPack;
class ContentIndices;
class ContentControl;
class Level;
class Block;
class Chunk;
class Player;
struct ItemDef;
class Inventory;
class UiDocument;
struct BlockFuncsSet;
struct ItemFuncsSet;
struct WorldFuncsSet;
struct UserComponent;
struct uidocscript;
class BlocksController;
class LevelController;
class Entity;
struct EntityDef;
class GeneratorScript;
struct GeneratorDef;
class Process;

namespace scripting {
    extern Engine* engine;
    extern const Content* content;
    extern const ContentIndices* indices;
    extern ContentControl* content_control;
    extern Level* level;
    extern BlocksController* blocks;
    extern LevelController* controller;
    extern std::ostream* output_stream;
    extern std::ostream* error_stream;

    void initialize(Engine* engine);

    void on_content_load(Content* content);
    void on_content_reset();

    bool register_event(
        int env, const std::string& name, const std::string& id
    );
    int get_values_on_stack();

    scriptenv get_root_environment();
    scriptenv create_pack_environment(const ContentPack& pack);
    scriptenv create_environment(const scriptenv& parent);
    scriptenv create_doc_environment(
        const scriptenv& parent, const std::string& name
    );

    void process_post_runnables();

    class IClientProjectScript {
    public:
        virtual ~IClientProjectScript() {}

        virtual void onScreenChange(const std::string& name, bool show) = 0;
    };

    std::unique_ptr<IClientProjectScript> load_client_project_script(
        const io::path& script
    );

    std::unique_ptr<Process> start_coroutine(const io::path& script);

    void on_world_load(LevelController* controller);
    void on_world_tick(int tps);
    void on_world_save();
    void on_world_quit();
    void cleanup();
    void on_blocks_tick(const Block& block, int tps);
    void update_block(const Block& block, const glm::ivec3& pos);
    void random_update_block(const Block& block, const glm::ivec3& pos);
    void on_block_placed(
        Player* player, const Block& block, const glm::ivec3& pos
    );
    void on_block_replaced(
        Player* player, const Block& block, const glm::ivec3& pos
    );
    void on_block_breaking(
        Player* player, const Block& block, const glm::ivec3& pos
    );
    void on_block_broken(
        Player* player, const Block& block, const glm::ivec3& pos
    );
    bool on_block_interact(Player* player, const Block& block, const glm::ivec3& pos);
    
    void on_chunk_present(const Chunk& chunk, bool loaded);
    void on_chunk_remove(const Chunk& chunk);

    void on_inventory_open(const Player* player, const Inventory& inventory);
    void on_inventory_closed(const Player* player, const Inventory& inventory);

    void on_player_tick(Player* player, int tps);

    /// @brief Called on RMB click with the item selected
    /// @return true if prevents default action
    bool on_item_use(Player* player, const ItemDef& item);

    /// @brief Called on RMB click on block with the item selected
    /// @return true if prevents default action
    bool on_item_use_on_block(
        Player* player, const ItemDef& item, glm::ivec3 ipos, glm::ivec3 normal
    );

    /// @brief Called on LMB click on block with the item selected
    /// @return true if prevents default action
    bool on_item_break_block(
        Player* player, const ItemDef& item, int x, int y, int z
    );

    dv::value get_component_value(
        const scriptenv& env, const std::string& name
    );
    void on_entity_spawn(
        const EntityDef& def,
        entityid_t eid,
        const std::vector<std::unique_ptr<UserComponent>>& components,
        const dv::value& args,
        const dv::value& saved
    );
    void on_entity_despawn(const Entity& entity);
    void on_entity_grounded(const Entity& entity, float force);
    void on_entity_fall(const Entity& entity);
    void on_entity_save(const Entity& entity);
    void on_entities_update(int tps, int parts, int part);
    void on_entities_physics_update(float delta);
    void on_entities_render(float delta);
    void on_sensor_enter(const Entity& entity, size_t index, entityid_t oid);
    void on_sensor_exit(const Entity& entity, size_t index, entityid_t oid);
    void on_aim_on(const Entity& entity, Player* player);
    void on_aim_off(const Entity& entity, Player* player);
    void on_attacked(const Entity& entity, Player* player, entityid_t attacker);
    void on_entity_used(const Entity& entity, Player* player);

    /// @brief Called on UI view show
    void on_ui_open(UiDocument* layout, std::vector<dv::value> args);

    void on_ui_progress(UiDocument* layout, int workDone, int totalWork);

    /// @brief Called on UI view close
    void on_ui_close(UiDocument* layout, Inventory* inventory);

    /// @brief Load script associated with a Block
    /// @param env environment
    /// @param prefix pack id
    /// @param file item script file
    /// @param fileName script file path using the engine format
    /// @param funcsset block callbacks set
    void load_content_script(
        const scriptenv& env,
        const std::string& prefix,
        const io::path& file,
        const std::string& fileName,
        BlockFuncsSet& funcsset
    );

    /// @brief Load script associated with an Item
    /// @param env environment
    /// @param prefix pack id
    /// @param file item script file
    /// @param fileName script file path using the engine format
    /// @param funcsset item callbacks set
    void load_content_script(
        const scriptenv& env,
        const std::string& prefix,
        const io::path& file,
        const std::string& fileName,
        ItemFuncsSet& funcsset
    );

    /// @brief Load component script
    /// @param name component full name (packid:name)
    /// @param file component script file path
    /// @param fileName script file path using the engine format
    void load_entity_component(
        const std::string& name,
        const io::path& file,
        const std::string& fileName
    );

    std::unique_ptr<GeneratorScript> load_generator(
        const GeneratorDef& def,
        const io::path& file,
        const std::string& dirPath
    );

    /// @brief Load package-specific world script
    /// @param env environment
    /// @param packid content-pack id
    /// @param file script file path
    /// @param fileName script file path using the engine format
    void load_world_script(
        const scriptenv& env,
        const std::string& packid,
        const io::path& file,
        const std::string& fileName,
        WorldFuncsSet& funcsset
    );

    /// @brief Load script associated with an UiDocument
    /// @param env environment
    /// @param prefix pack id
    /// @param file item script file
    /// @param fileName script file path using the engine format
    /// @param script document script info
    void load_layout_script(
        const scriptenv& env,
        const std::string& prefix,
        const io::path& file,
        const std::string& fileName,
        uidocscript& script
    );

    /// @brief Finalize lua state. Using scripting after will lead to Lua panic
    void close();
}
