#pragma once

#include "typedefs.hpp"
#include "util/ObjectsKeeper.hpp"
#include "data/dv.hpp"

#include <string>
#include <memory>
#include <vector>
#include <glm/glm.hpp>

class Camera;
class Block;
class Assets;
class Player;
class Engine;
class Inventory;
class LevelFrontend;
class UiDocument;
class DrawContext;
class ImageData;
class Input;

namespace gui {
    class GUI;
    class Menu;
    class UINode;
    class Panel;
    class Container;
    class InventoryView;
    class SlotView;
}

enum class HudElementMode {
    // element is hidden if menu or inventory open
    INGAME,
    // element is visible if hud is visible
    PERMANENT,
    // element is visible in inventory mode
    INVENTORY_ANY,
    // element will be removed on inventory close
    INVENTORY
};

class HudElement {
    HudElementMode mode;
    UiDocument* document;
    std::shared_ptr<gui::UINode> node;

    bool debug;
    bool removed = false;
public:
    HudElement(HudElementMode mode, UiDocument* document, std::shared_ptr<gui::UINode> node, bool debug);

    void update(bool pause, bool inventoryOpen, bool debug);

    UiDocument* getDocument() const;
    std::shared_ptr<gui::UINode> getNode() const;

    bool isInventoryBound() const {
        return mode == HudElementMode::INVENTORY;
    }

    void setRemoved() {
        removed = true;
    }

    bool isRemoved() const {
        return removed;
    }
};

class Hud : public util::ObjectsKeeper {
    Engine& engine;
    Input& input;
    Assets& assets;
    gui::GUI& gui;
    gui::Menu& menu;
    std::unique_ptr<Camera> uicamera;
    LevelFrontend& frontend;
    Player& player;

    /// @brief Is any overlay/inventory open
    bool inventoryOpen = false;
    /// @brief Is pause mode on
    bool pause = false;

    /// @brief Content access panel scroll container
    std::shared_ptr<gui::Container> contentAccessPanel;
    /// @brief Content access panel itself
    std::shared_ptr<gui::InventoryView> contentAccess;
    /// @brief Player inventory hotbar
    std::shared_ptr<gui::InventoryView> hotbarView;
    /// @brief Debug info and control panel (F3 key)
    std::shared_ptr<gui::UINode> debugPanel;
    /// @brief Overlay used in pause mode
    std::shared_ptr<gui::UINode> darkOverlay;
    /// @brief Inventories interaction agent (grabbed item)
    std::shared_ptr<gui::SlotView> exchangeSlot;
    /// @brief Exchange slot inventory (1 slot only)
    std::shared_ptr<Inventory> exchangeSlotInv;
    /// @brief List of all controlled hud elements
    std::vector<HudElement> elements;

    /// @brief Player inventory view
    std::shared_ptr<gui::InventoryView> inventoryView;
    /// @brief Block inventory view
    std::shared_ptr<gui::InventoryView> blockUI;
    /// @brief Secondary inventory view
    std::shared_ptr<gui::InventoryView> secondInvView;
    /// @brief Position of the block open
    glm::ivec3 blockPos {};
    /// @brief Id of the block open (used to detect block destruction or replacement)
    blockid_t currentblockid = 0;
    /// @brief Show content access panel
    bool showContentPanel = true;
    /// @brief Provide cheat controllers to the debug panel
    bool allowDebugCheats = true;
    /// @brief Allow actual pause
    bool allowPause = true;
    bool debug = false;
    /// @brief UI element will be dynamicly positioned near to inventory or in screen center
    std::shared_ptr<gui::UINode> secondUI;

    std::shared_ptr<gui::UINode> debugMinimap;

    std::unique_ptr<ImageData> debugImgWorldGen;
    
    std::shared_ptr<gui::InventoryView> createContentAccess();
    std::shared_ptr<gui::InventoryView> createHotbar();

    void processInput(bool visible);
    void updateElementsPosition(const glm::uvec2& viewport);
    void updateHotbarControl();
    void cleanup();

    /// @brief Perform exchange slot removal when it's not empty.
    void dropExchangeSlot();

    void showExchangeSlot();
    void updateWorldGenDebug();
public:
    Hud(Engine& engine, LevelFrontend& frontend, Player& player);
    ~Hud();

    void update(bool hudVisible);
    void draw(const DrawContext& context);

    /// @brief Check if inventory mode on
    bool isInventoryOpen() const;

    /// @brief Check if pause mode on
    bool isPause() const;

    /// @brief Enable/disable pause mode
    void setPause(bool pause);

    /// @brief Show player inventory in inventory-mode
    void openInventory();

    /// @brief Show inventory in inventory-mode
    /// @param doc ui layout
    /// @param inv inventory
    /// @param playerInventory show player inventory too
    std::shared_ptr<Inventory> openInventory(
        UiDocument* doc,
        std::shared_ptr<Inventory> inv,
        bool playerInventory
    );
    
    /// @brief Show block inventory in inventory-mode
    /// @param block block position
    /// @param doc block ui layout
    /// @param blockInv block inventory
    /// @param playerInventory show player inventory too
    void openInventory(
        glm::ivec3 block, 
        UiDocument* doc, 
        std::shared_ptr<Inventory> blockInv, 
        bool playerInventory
    );

    /// @brief Show element in inventory-mode
    /// @param doc element layout
    /// @param playerInventory show player inventory too
    /// @param arg first argument passing to on_open
    void showOverlay(
        UiDocument* doc, bool playerInventory, const dv::value& arg = nullptr
    );

    /// @brief Close all open inventories and overlay
    void closeInventory();

    /// @brief Add element will be visible until removed
    /// @param doc element layout
    void openPermanent(UiDocument* doc);

    void add(const HudElement& element, const dv::value& arg=nullptr);
    void onRemove(const HudElement& element);
    void remove(const std::shared_ptr<gui::UINode>& node);

    void setDebug(bool flag);

    Player* getPlayer() const;

    std::shared_ptr<Inventory> getBlockInventory();

    bool isContentAccess() const;

    void setContentAccess(bool flag);

    void setDebugCheats(bool flag);

    void setAllowPause(bool flag);

    bool isOpen(const std::string& layoutid) const;

    static bool showGeneratorMinimap;

    /// @brief Runtime updating debug visualization texture
    inline static std::string DEBUG_WORLDGEN_IMAGE = "#debug.img.worldgen";
};
