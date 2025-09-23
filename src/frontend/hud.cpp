#include "hud.hpp"

#include "ContentGfxCache.hpp"
#include "LevelFrontend.hpp"
#include "UiDocument.hpp"

#include "assets/Assets.hpp"
#include "content/Content.hpp"
#include "core_defs.hpp"
#include "delegates.hpp"
#include "engine/Engine.hpp"
#include "graphics/core/Atlas.hpp"
#include "graphics/core/Batch2D.hpp"
#include "graphics/core/Batch3D.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/Texture.hpp"
#include "graphics/core/ImageData.hpp"
#include "graphics/render/WorldRenderer.hpp"
#include "graphics/ui/elements/InventoryView.hpp"
#include "graphics/ui/elements/Menu.hpp"
#include "graphics/ui/elements/Panel.hpp"
#include "graphics/ui/elements/Plotter.hpp"
#include "graphics/ui/elements/UINode.hpp"
#include "graphics/ui/gui_util.hpp"
#include "graphics/ui/GUI.hpp"
#include "items/Inventories.hpp"
#include "items/Inventory.hpp"
#include "items/ItemDef.hpp"
#include "logic/scripting/scripting.hpp"
#include "logic/LevelController.hpp"
#include "world/generator/WorldGenerator.hpp"
#include "maths/voxmaths.hpp"
#include "objects/Player.hpp"
#include "physics/Hitbox.hpp"
#include "typedefs.hpp"
#include "util/stringutil.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/GlobalChunks.hpp"
#include "window/Camera.hpp"
#include "window/input.hpp"
#include "window/Window.hpp"
#include "world/Level.hpp"
#include "world/World.hpp"
#include "debug/Logger.hpp"

#include <assert.h>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

using namespace gui;

static debug::Logger logger("hud");

bool Hud::showGeneratorMinimap = false;

// implemented in debug_panel.cpp
std::shared_ptr<UINode> create_debug_panel(
    Engine& engine,
    Level& level,
    Player& player,
    bool allowDebugCheats
);

HudElement::HudElement(
    HudElementMode mode, 
    UiDocument* document, 
    std::shared_ptr<UINode> node, 
    bool debug
) : mode(mode), document(document), node(std::move(node)), debug(debug) {
}

void HudElement::update(bool pause, bool inventoryOpen, bool debugMode) {
    if (debug && !debugMode) {
        node->setVisible(false);
        return;
    }
    switch (mode) {
        case HudElementMode::PERMANENT:
            node->setVisible(true);
            break;
        case HudElementMode::INGAME:
            node->setVisible(!pause && !inventoryOpen);
            break;
        case HudElementMode::INVENTORY_ANY:
            node->setVisible(inventoryOpen);
            break;
        case HudElementMode::INVENTORY:
            removed = !inventoryOpen;
            break;
    }
}

UiDocument* HudElement::getDocument() const {
    return document;
}

std::shared_ptr<UINode> HudElement::getNode() const {
    return node;
}

std::shared_ptr<InventoryView> Hud::createContentAccess() {
    auto& content = frontend.getLevel().content;
    auto& indices = *content.getIndices();
    auto inventory = player.getInventory();
    
    size_t itemsCount = indices.items.count();
    auto accessInventory = std::make_shared<Inventory>(0, itemsCount);
    for (size_t id = 1; id < itemsCount; id++) {
        accessInventory->getSlot(id-1).set(ItemStack(id, 1));
    }

    SlotLayout slotLayout(-1, glm::vec2(), false, true, nullptr,
    [inventory, &indices](uint, ItemStack& item) {
        auto copy = ItemStack(item);
        inventory->move(copy, indices);
    }, 
    [this, inventory](uint, ItemStack& item) {
        inventory->getSlot(player.getChosenSlot()).set(item);
    });

    InventoryBuilder builder(gui);
    builder.addGrid(8, itemsCount-1, glm::vec2(), glm::vec4(8, 8, 12, 8), true, slotLayout);
    auto view = builder.build();
    view->bind(accessInventory, &content);
    view->setMargin(glm::vec4());
    return view;
}

std::shared_ptr<InventoryView> Hud::createHotbar() {
    auto inventory = player.getInventory();
    auto& content = frontend.getLevel().content;

    SlotLayout slotLayout(-1, glm::vec2(), false, false, nullptr, nullptr, nullptr);
    InventoryBuilder builder(gui);
    builder.addGrid(10, 10, glm::vec2(), glm::vec4(4), true, slotLayout);
    auto view = builder.build();
    view->setId("hud.hotbar");
    view->setOrigin(glm::vec2(view->getSize().x/2, 0));
    view->bind(inventory, &content);
    view->setInteractive(false);
    return view;
}

static constexpr uint WORLDGEN_IMG_SIZE = 128U;

Hud::Hud(Engine& engine, LevelFrontend& frontend, Player& player)
    : engine(engine),
      input(engine.getInput()),
      assets(*engine.getAssets()),
      gui(engine.getGUI()),
      menu(*engine.getGUI().getMenu()),
      frontend(frontend),
      player(player),
      debugImgWorldGen(std::make_unique<ImageData>(
          ImageFormat::rgba8888, WORLDGEN_IMG_SIZE, WORLDGEN_IMG_SIZE
      )) {
    contentAccess = createContentAccess();
    contentAccess->setId("hud.content-access");
    contentAccessPanel = std::make_shared<Panel>(
        gui, contentAccess->getSize(), glm::vec4(0.0f), 0.0f
    );
    contentAccessPanel->setColor(glm::vec4());
    contentAccessPanel->add(contentAccess);
    contentAccessPanel->setScrollable(true);
    contentAccessPanel->setId("hud.content-access-panel");

    hotbarView = createHotbar();
    darkOverlay = guiutil::create(
        gui,
        "<container size='4000' color='#00000080' z-index='-1' visible='false'/>"
    );

    uicamera = std::make_unique<Camera>(glm::vec3(), 1);
    uicamera->perspective = false;
    uicamera->flipped = true;
    uicamera->near = -1.0f;
    uicamera->far = 1.0f;

    debugPanel = create_debug_panel(
        engine, frontend.getLevel(), player, allowDebugCheats
    );
    debugPanel->setZIndex(2);

    gui.add(debugPanel);
    gui.add(darkOverlay);
    gui.add(hotbarView);
    gui.add(contentAccessPanel);

    auto dplotter = std::make_shared<Plotter>(gui, 350, 250, 2000, 16);
    dplotter->setGravity(Gravity::bottom_right);
    dplotter->setInteractive(false);
    add(HudElement(HudElementMode::PERMANENT, nullptr, dplotter, true));

    assets.store(Texture::from(debugImgWorldGen.get()), DEBUG_WORLDGEN_IMAGE);

    debugMinimap = guiutil::create(
        gui,
        "<image src='" + DEBUG_WORLDGEN_IMAGE +
            "' pos='0' size='256' gravity='top-right' margin='0,20,0,0'/>"
    );
    add(HudElement(HudElementMode::PERMANENT, nullptr, debugMinimap, true));
}

Hud::~Hud() {
    if (input.isCursorLocked()) {
        input.toggleCursor();
    }
    // removing all controlled ui
    for (auto& element : elements) {
        onRemove(element);
    }
    gui.remove(hotbarView);
    gui.remove(darkOverlay);
    gui.remove(contentAccessPanel);
    gui.remove(debugPanel);
}

/// @brief Remove all elements marked as removed
void Hud::cleanup() {
    auto it = std::remove_if(elements.begin(), elements.end(), [](const HudElement& e) {
        return e.isRemoved();
    });
    elements.erase(it, elements.end());
}

void Hud::processInput(bool visible) {
    const auto& window = engine.getWindow();
    if (!window.isFocused() && !menu.hasOpenPage() && !isInventoryOpen()) {
        setPause(true);
    }
    const auto& bindings = input.getBindings();
    if (!pause && visible && bindings.jactive(BIND_HUD_INVENTORY)) {
        if (inventoryOpen) {
            closeInventory();
        } else {
            openInventory();
        }
    }
    if (!pause) {
        updateHotbarControl();
    }
}

void Hud::updateHotbarControl() {
    int scroll = input.getScroll();
    if (!inventoryOpen && scroll) {
        int slot = player.getChosenSlot();
        slot = (slot - scroll) % 10;
        if (slot < 0) {
            slot += 10;
        }
        player.setChosenSlot(slot);
    }
    for (
        int i = static_cast<int>(Keycode::NUM_1); 
        i <= static_cast<int>(Keycode::NUM_9); 
        i++
    ) {
        if (input.jpressed(static_cast<Keycode>(i))) {
            player.setChosenSlot(i - static_cast<int>(Keycode::NUM_1));
        }
    }
    if (input.jpressed(Keycode::NUM_0)) {
        player.setChosenSlot(9);
    }
}

void Hud::updateWorldGenDebug() {
    auto& level = frontend.getLevel();
    const auto& chunks = *player.chunks;
    uint padding = engine.getSettings().chunks.padding.get();
    auto generator =
        frontend.getController()->getChunksController()->getGenerator();
    auto debugInfo = generator->createDebugInfo();
    
    int width = debugImgWorldGen->getWidth();
    int height = debugImgWorldGen->getHeight();
    ubyte* data = debugImgWorldGen->getData();

    int ox = debugInfo.areaOffsetX;
    int oz = debugInfo.areaOffsetY;

    int areaWidth = debugInfo.areaWidth;
    int areaHeight = debugInfo.areaHeight;

    for (int z = 0; z < height; z++) {
        for (int x = 0; x < width; x++) {
            int flippedZ = height - z - 1;

            int ax = x - (width - areaWidth) / 2;
            int az = z - (height - areaHeight) / 2;

            bool isInLoadingZone =
                frontend.getController()
                    ->getChunksController()
                    ->isInLoadingZone(player, padding, ax + ox, az + oz);

            data[(flippedZ * width + x) * 4 + 1] =
                chunks.getChunk(ax + ox, az + oz)
                    ? (isInLoadingZone ? 255 : 128)
                    : 0;
            data[(flippedZ * width + x) * 4 + 0] = 
                level.chunks->fetch(ax + ox, az + oz) ? 255 : 0;

            if (ax < 0 || az < 0 || 
                ax >= areaWidth || az >= areaHeight) {
                data[(flippedZ * width + x) * 4 + 2] = 0;
                data[(flippedZ * width + x) * 4 + 3] = 0;
                data[(flippedZ * width + x) * 4 + 3] = 100;
                continue;
            }
            auto value = debugInfo.areaLevels[az * areaWidth + ax] * 25;

            // Chunk is already generated
            data[(flippedZ * width + x) * 4 + 2] = value;
            data[(flippedZ * width + x) * 4 + 3] = 150;
        }
    }
    auto& texture = assets.require<Texture>(DEBUG_WORLDGEN_IMAGE);
    texture.reload(*debugImgWorldGen);
}

void Hud::update(bool visible) {
    const auto& chunks = *player.chunks;
    bool isMenuOpen = menu.hasOpenPage();

    debugPanel->setVisible(
        debug && visible && !(inventoryOpen && inventoryView == nullptr)
    );

    if (!visible && inventoryOpen) {
        closeInventory();
    }
    if (pause && !isMenuOpen) {
        setPause(false);
    }
    if (!gui.isFocusCaught()) {
        processInput(visible);
    }
    if ((isMenuOpen || inventoryOpen) == input.isCursorLocked()) {
        input.toggleCursor();
    }

    if (blockUI) {
        voxel* vox = chunks.get(blockPos.x, blockPos.y, blockPos.z);
        if (vox == nullptr || vox->id != currentblockid) {
            closeInventory();
        }
    }

    for (auto& element : elements) {
        element.getNode()->setVisible(visible);
    }

    const auto& windowSize = engine.getWindow().getSize();
    glm::vec2 caSize = contentAccessPanel->getSize();
    contentAccessPanel->setVisible(inventoryView != nullptr && showContentPanel);
    contentAccessPanel->setSize(glm::vec2(caSize.x, windowSize.y));
    contentAccess->setMinSize(glm::vec2(1, windowSize.y));
    hotbarView->setVisible(visible && !(secondUI && !inventoryView));
    darkOverlay->setVisible(isMenuOpen);
    menu.setVisible(isMenuOpen);

    if (visible) {
        for (auto& element : elements) {
            element.update(pause, inventoryOpen, debug);
            if (element.isRemoved()) {
                onRemove(element);
            }
        }
    }
    cleanup();

    debugMinimap->setVisible(debug && showGeneratorMinimap && visible);
    if (debug && showGeneratorMinimap) {
        updateWorldGenDebug();
    }
}

/// @brief Show inventory on the screen and turn on inventory mode blocking movement
void Hud::openInventory() {
    auto& content = frontend.getLevel().content;
    showExchangeSlot();

    inventoryOpen = true;
    auto inventory = player.getInventory();
    auto inventoryDocument = assets.get<UiDocument>("core:inventory");
    inventoryView = std::dynamic_pointer_cast<InventoryView>(inventoryDocument->getRoot());
    inventoryView->bind(inventory, &content);
    add(HudElement(HudElementMode::INVENTORY, inventoryDocument, inventoryView, false));
    add(HudElement(HudElementMode::INVENTORY, nullptr, exchangeSlot, false));
}

std::shared_ptr<Inventory> Hud::openInventory(
    UiDocument* doc,
    std::shared_ptr<Inventory> inv,
    bool playerInventory
) {
    if (isInventoryOpen()) {
        closeInventory();
    }
    const auto& level = frontend.getLevel();
    auto& content = level.content;
    secondInvView = std::dynamic_pointer_cast<InventoryView>(doc->getRoot());
    if (secondInvView == nullptr) {
        throw std::runtime_error("secondary UI root element must be 'inventory'");
    }
    secondUI = secondInvView;

    if (playerInventory) {
        openInventory();
    } else {
        inventoryOpen = true;
    }
    if (inv == nullptr) {
        inv = level.inventories->createVirtual(secondInvView->getSlotsCount());
    }
    secondInvView->bind(inv, &content);
    add(HudElement(HudElementMode::INVENTORY, doc, secondUI, false));
    scripting::on_inventory_open(&player, *inv);
    return inv;
}

void Hud::openInventory(
    glm::ivec3 block, 
    UiDocument* doc, 
    std::shared_ptr<Inventory> blockinv, 
    bool playerInventory
) {
    if (isInventoryOpen()) {
        closeInventory();
    }
    auto& level = frontend.getLevel();
    const auto& chunks = *player.chunks;
    auto& content = level.content;

    blockUI = std::dynamic_pointer_cast<InventoryView>(doc->getRoot());
    if (blockUI == nullptr) {
        throw std::runtime_error("block UI root element must be 'inventory'");
    }
    secondUI = blockUI;
    if (playerInventory) {
        openInventory();
    } else {
        inventoryOpen = true;
    }
    if (blockinv == nullptr) {
        blockinv = level.inventories->createVirtual(blockUI->getSlotsCount());
    }
    chunks.getChunkByVoxel(block)->flags.unsaved = true;
    blockUI->bind(blockinv, &content);
    blockPos = block;
    currentblockid = chunks.require(block.x, block.y, block.z).id;
    add(HudElement(HudElementMode::INVENTORY, doc, blockUI, false));

    scripting::on_inventory_open(&player, *blockinv);
}

void Hud::showExchangeSlot() {
    auto& level = frontend.getLevel();
    auto& content = level.content;
    exchangeSlotInv = level.inventories->createVirtual(1);
    exchangeSlot = std::make_shared<SlotView>(
        gui,
        SlotLayout(-1, glm::vec2(), false, false, nullptr, nullptr, nullptr)
    );
    exchangeSlot->bind(exchangeSlotInv->getId(), exchangeSlotInv->getSlot(0), &content);
    exchangeSlot->setColor(glm::vec4());
    exchangeSlot->setInteractive(false);
    exchangeSlot->setZIndex(1);
    gui.store(SlotView::EXCHANGE_SLOT_NAME, exchangeSlot);
}

void Hud::showOverlay(
    UiDocument* doc, bool playerInventory, const dv::value& args
) {
    if (isInventoryOpen()) {
        closeInventory();
    }
    secondUI = doc->getRoot();
    if (playerInventory) {
        openInventory();
    } else {
        showExchangeSlot();
        inventoryOpen = true;
    }
    add(HudElement(HudElementMode::INVENTORY, doc, secondUI, false), args);
}

void Hud::openPermanent(UiDocument* doc) {
    auto root = doc->getRoot();
    remove(root);

    auto invview = std::dynamic_pointer_cast<InventoryView>(root);
    if (invview) {
        invview->bind(player.getInventory(), &frontend.getLevel().content);
    }
    add(HudElement(HudElementMode::PERMANENT, doc, doc->getRoot(), false));
}

void Hud::dropExchangeSlot() {
    auto slotView = std::dynamic_pointer_cast<SlotView>(
        gui.get(SlotView::EXCHANGE_SLOT_NAME)
    );
    if (slotView == nullptr) {
        return;
    }
    ItemStack& stack = slotView->getStack();
    
    auto indices = frontend.getLevel().content.getIndices();
    if (auto invView = std::dynamic_pointer_cast<InventoryView>(blockUI)) {
        invView->getInventory()->move(stack, *indices);
    }
    if (stack.isEmpty()) {
        return;
    }
    player.getInventory()->move(stack, *indices);
    if (!stack.isEmpty()) {
        logger.warning() << "discard item [" << stack.getItemId() << ":"
                         << stack.getCount();
        stack.clear();
    }
}

void Hud::closeInventory() {
    if (blockUI) {
        scripting::on_inventory_closed(&player, *blockUI->getInventory());
        blockUI = nullptr;
    }
    if (secondInvView) {
        scripting::on_inventory_closed(&player, *secondInvView->getInventory());
    }
    dropExchangeSlot();
    gui.remove(SlotView::EXCHANGE_SLOT_NAME);
    exchangeSlot = nullptr;
    exchangeSlotInv = nullptr;
    inventoryOpen = false;
    inventoryView = nullptr;
    secondInvView = nullptr;
    secondUI = nullptr;

    for (auto& element : elements) {
        if (element.isInventoryBound()) {
            element.setRemoved();
            onRemove(element);
        }
    }
    cleanup();
}

void Hud::add(const HudElement& element, const dv::value& argsArray) {
    gui.add(element.getNode());
    auto document = element.getDocument();
    if (document) {
        auto invview = std::dynamic_pointer_cast<InventoryView>(element.getNode());
        auto inventory = invview ? invview->getInventory() : nullptr;
        std::vector<dv::value> args;
        if (argsArray != nullptr) {
            for (const auto& arg : argsArray) {
                args.push_back(arg);
            }
        }
        args.emplace_back(inventory ? inventory.get()->getId() : 0);
        for (int i = 0; i < 3; i++) {
            args.emplace_back(static_cast<integer_t>(blockPos[i]));
        }
        scripting::on_ui_open(
            element.getDocument(), 
            std::move(args)
        );
    }
    elements.push_back(element);
}

void Hud::onRemove(const HudElement& element) {
    auto document = element.getDocument();
    if (document) {
        Inventory* inventory = nullptr;
        auto invview = std::dynamic_pointer_cast<InventoryView>(element.getNode());
        if (invview) {
            inventory = invview->getInventory().get();
        }
        scripting::on_ui_close(document, inventory);
        if (invview) {
            invview->unbind();
        }
    }
    gui.remove(element.getNode());
}

void Hud::remove(const std::shared_ptr<UINode>& node) {
    for (auto& element : elements) {
        if (element.getNode() == node) {
            element.setRemoved();
            onRemove(element);
        }
    }
    cleanup();
    if (node == secondUI) {
        closeInventory();
    }
}

void Hud::setDebug(bool flag) {
    debug = flag;
}

void Hud::draw(const DrawContext& ctx){
    const auto& viewport = ctx.getViewport();

    updateElementsPosition(viewport);

    uicamera->setFov(viewport.y);
    uicamera->setAspectRatio(viewport.x / static_cast<float>(viewport.y));

    auto batch = ctx.getBatch2D();
    batch->begin();

    auto& uishader = assets.require<Shader>("ui");
    uishader.use();
    uishader.uniformMatrix("u_projview", uicamera->getProjView());

    // Crosshair
    if (!pause && !inventoryOpen && !debug) {
        DrawContext chctx = ctx.sub(batch);
        chctx.setBlendMode(BlendMode::inversion);
        auto texture = assets.get<Texture>("gui/crosshair");
        batch->texture(texture);
        int chsizex = texture != nullptr ? texture->getWidth() : 16;
        int chsizey = texture != nullptr ? texture->getHeight() : 16;
        batch->rect(
            (viewport.x - chsizex) / 2, (viewport.y - chsizey) / 2, 
            chsizex, chsizey, 0, 0, 1, 1, 1, 1, 1, 1
        );
    }
}

void Hud::updateElementsPosition(const glm::uvec2& viewport) {
    if (inventoryOpen) {
        float caWidth = inventoryView && showContentPanel
                            ? contentAccess->getSize().x
                            : 0.0f;
        contentAccessPanel->setPos(glm::vec2(viewport.x - caWidth, 0));

        glm::vec2 invSize = inventoryView ? inventoryView->getSize() : glm::vec2();
        if (secondUI == nullptr) {
            if (inventoryView) {
                inventoryView->setPos(glm::vec2(
                    glm::min(
                        viewport.x / 2 - invSize.x / 2,
                        viewport.x - caWidth - 10 - invSize.x
                    ),
                    viewport.y / 2 - invSize.y / 2
                ));
            }
        } else {
            glm::vec2 secondUISize = secondUI->getSize();
            float invwidth = glm::max(invSize.x, secondUISize.x);
            int interval = invSize.y > 0.0 ? 5 : 0;
            float totalHeight = invSize.y + secondUISize.y + interval;
            if (inventoryView) {
                inventoryView->setPos(glm::vec2(
                    glm::min(
                        viewport.x / 2 - invwidth / 2,
                        viewport.x - caWidth - 10 - invwidth
                    ),
                    viewport.y / 2 + totalHeight / 2 - invSize.y
                ));
            }
            if (secondUI->getPositionFunc() == nullptr) {
                secondUI->setPos(glm::vec2(
                    glm::min(
                        viewport.x / 2.f - invwidth / 2.f,
                        viewport.x - caWidth - (inventoryView ? 10 : 0) -
                            invwidth
                    ),
                    viewport.y / 2.f - totalHeight / 2.f
                ));
            }
        }
    }
    if (exchangeSlot != nullptr) {
        exchangeSlot->setPos(input.getCursor().pos);
    }
    hotbarView->setPos(glm::vec2(viewport.x / 2, viewport.y - 65));
    hotbarView->setSelected(player.getChosenSlot());
}

bool Hud::isInventoryOpen() const {
    return inventoryOpen;
}

bool Hud::isPause() const {
    return pause;
}

void Hud::setPause(bool pause) {
    if (this->pause == pause) {
        return;
    }
    if (allowPause) {
        this->pause = pause;
    }

    if (inventoryOpen) {
        closeInventory();
    }
    
    if (!pause && menu.hasOpenPage()) {
        menu.reset();
    }
    if (pause && !menu.hasOpenPage()) {
        menu.setPage("pause");
    }
}

Player* Hud::getPlayer() const {
    return &player;
}

std::shared_ptr<Inventory> Hud::getBlockInventory() {
    if (blockUI == nullptr) {
        return nullptr;
    }
    return blockUI->getInventory();
}

bool Hud::isContentAccess() const {
    return showContentPanel;
}

void Hud::setContentAccess(bool flag) {
    showContentPanel = flag;
}

void Hud::setDebugCheats(bool flag) {
    allowDebugCheats = flag;
    
    gui.remove(debugPanel);
    debugPanel = create_debug_panel(
        engine, frontend.getLevel(), player, allowDebugCheats
    );
    debugPanel->setZIndex(2);
    gui.add(debugPanel);
}

void Hud::setAllowPause(bool flag) {
    if (pause) {
        setPause(false);
        menu.setPage("pause", true);
    }
    allowPause = flag;
}
