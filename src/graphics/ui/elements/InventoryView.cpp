#include "InventoryView.hpp"

#include <glm/glm.hpp>
#include <utility>

#include "assets/Assets.hpp"
#include "assets/assets_util.hpp"
#include "content/Content.hpp"
#include "frontend/LevelFrontend.hpp"
#include "frontend/locale.hpp"
#include "graphics/core/Atlas.hpp"
#include "graphics/core/Batch2D.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/Font.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/Texture.hpp"
#include "graphics/render/BlocksPreview.hpp"
#include "graphics/ui/GUI.hpp"
#include "items/Inventories.hpp"
#include "items/Inventory.hpp"
#include "items/ItemDef.hpp"
#include "logic/scripting/scripting.hpp"
#include "maths/voxmaths.hpp"
#include "objects/Player.hpp"
#include "util/stringutil.hpp"
#include "voxels/Block.hpp"
#include "window/input.hpp"
#include "world/Level.hpp"

using namespace gui;

SlotLayout::SlotLayout(
    int index,
    glm::vec2 position,
    bool background,
    bool itemSource,
    slotcallback updateFunc,
    slotcallback shareFunc,
    slotcallback rightClick
)
    : index(index),
      position(position),
      background(background),
      itemSource(itemSource),
      updateFunc(std::move(updateFunc)),
      shareFunc(std::move(shareFunc)),
      rightClick(std::move(rightClick)) {
}

InventoryBuilder::InventoryBuilder(GUI& gui) : gui(gui) {
    view = std::make_shared<InventoryView>(gui);
}

void InventoryBuilder::addGrid(
    int cols,
    int count,
    glm::vec2 pos,
    glm::vec4 padding,
    bool addpanel,
    const SlotLayout& slotLayout
) {
    const int slotSize = InventoryView::SLOT_SIZE;
    const int interval = InventoryView::SLOT_INTERVAL;

    int rows = ceildiv(count, cols);

    uint width =
        cols * (slotSize + interval) - interval + padding.x + padding.z;
    uint height =
        rows * (slotSize + interval) - interval + padding.y + padding.w;

    glm::vec2 vsize = view->getSize();
    if (pos.x + width > vsize.x) {
        vsize.x = pos.x + width;
    }
    if (pos.y + height > vsize.y) {
        vsize.y = pos.y + height;
    }
    view->setSize(vsize);

    if (addpanel) {
        auto panel =
            std::make_shared<gui::Container>(gui, glm::vec2(width, height));
        view->setColor(glm::vec4(0.122f, 0.122f, 0.122f, 0.878f));
        view->add(panel, pos);
    }

    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            if (row * cols + col >= count) {
                break;
            }
            glm::vec2 position(
                col * (slotSize + interval) + padding.x,
                row * (slotSize + interval) + padding.y
            );
            auto builtSlot = slotLayout;
            builtSlot.index = row * cols + col;
            builtSlot.position = position;
            add(builtSlot);
        }
    }
}

void InventoryBuilder::add(const SlotLayout& layout) {
    view->add(view->addSlot(layout), layout.position);
}

std::shared_ptr<InventoryView> InventoryBuilder::build() {
    return view;
}

SlotView::SlotView(GUI& gui, SlotLayout layout)
    : UINode(gui, glm::vec2(InventoryView::SLOT_SIZE)),
      layout(std::move(layout)) {
    setColor(glm::vec4(0, 0, 0, 0.2f));
    setTooltipDelay(0.0f);
}
// TODO: Refactor
static std::wstring get_caption_string(
    const ItemStack& stack, const ItemDef& item
) {
    dv::value* caption = stack.getField("caption");
    if (caption != nullptr) {
        return util::pascal_case(
            langs::get(util::str2wstr_utf8(caption->asString()))
        );
    } else {
        return util::pascal_case(
            langs::get(util::str2wstr_utf8(item.caption))
        );
    }
}
// TODO: Refactor
static std::wstring get_description_string(
    const ItemStack& stack, const ItemDef& item
) {
    dv::value* description = stack.getField("description");

    if (description != nullptr) {
        return langs::get(util::str2wstr_utf8(description->asString()));
    } else {
        return langs::get(util::str2wstr_utf8(item.description));
    }
}

static bool is_same_tooltip(const ItemStack& stack, const ItemStack& cache) {
    if (stack.getItemId() != cache.getItemId()) {
        return false;
    }
    auto caption = stack.getField("caption");
    auto cCaption = cache.getField("caption");
    auto description = stack.getField("description");
    auto cDescription = cache.getField("description");

    if (((caption != nullptr) != (cCaption != nullptr)) ||
        ((description != nullptr) != (cDescription != nullptr))) {
        return false;
    }
    return (caption ? caption->asString() == cCaption->asString() : true) &&
           (description ? description->asString() == cDescription->asString()
                        : true);
}

void SlotView::refreshTooltip(const ItemStack& stack, const ItemDef& item) {
    itemid_t itemid = stack.getItemId();

    if (is_same_tooltip(stack, cache.stack)) {
        return;
    }
    if (itemid) {
        std::wstring caption = get_caption_string(stack, item);
        std::wstring description = get_description_string(stack, item);
        if (description.length() > 0) {
            tooltip = caption + L"\n" + description;
        } else {
            tooltip = caption;
        }
    } else {
        tooltip.clear();
    }
}

void SlotView::drawItemIcon(
    Batch2D& batch,
    const ItemStack& stack,
    const ItemDef& item,
    const Assets& assets,
    const glm::vec4& tint,
    const glm::vec2& pos
) {
    const int SLOT_SIZE = InventoryView::SLOT_SIZE;
    const auto& previews = assets.require<Atlas>("block-previews");
    batch.setColor(glm::vec4(1.0f));
    switch (item.iconType) {
        case ItemIconType::NONE:
            break;
        case ItemIconType::BLOCK: {
            const Block& block = content->blocks.require(item.icon);
            batch.texture(previews.getTexture());

            UVRegion region = previews.get(block.name);
            batch.rect(
                pos.x,
                pos.y,
                SLOT_SIZE,
                SLOT_SIZE,
                0,
                0,
                0,
                region,
                false,
                true,
                tint
            );
            break;
        }
        case ItemIconType::SPRITE: {
            auto textureRegion =
                util::get_texture_region(assets, item.icon, "blocks:notfound");

            batch.texture(textureRegion.texture);
            batch.rect(
                pos.x,
                pos.y,
                SLOT_SIZE,
                SLOT_SIZE,
                0,
                0,
                0,
                textureRegion.region,
                false,
                true,
                tint
            );
            break;
        }
    }
}

void SlotView::draw(const DrawContext& pctx, const Assets& assets) {
    if (bound == nullptr) {
        return;
    }
    const auto& indices = *content->getIndices();
    const ItemStack& stack = *bound;
    const ItemDef& item = indices.items.require(stack.getItemId());

    if (cache.stack.getCount() != stack.getCount()) {
        cache.countStr = std::to_wstring(stack.getCount());
    }
    refreshTooltip(stack, item);
    cache.stack.set(stack);

    glm::vec4 tint(1, 1, 1, isEnabled() ? 1 : 0.5f);
    glm::vec2 pos = calcPos();
    glm::vec4 color = getColor();

    if (hover || highlighted) {
        tint *= 1.333f;
        color = glm::vec4(1, 1, 1, 0.2f);
    }

    auto& batch = *pctx.getBatch2D();

    if (color.a > 0.0) {
        batch.setColor(color);
        batch.texture(nullptr);

        const int size = InventoryView::SLOT_SIZE;
        if (highlighted) {
            batch.rect(pos.x - 4, pos.y - 4, size + 8, size + 8);
        } else {
            batch.rect(pos.x, pos.y, size, size);
        }
    }

    drawItemIcon(batch, stack, item, assets, tint, pos);

    if (stack.getCount() > 1 || stack.getFields() != nullptr) {
        const auto& font = assets.require<Font>(FONT_DEFAULT);
        drawItemInfo(batch, stack, item, font, pos);
    }
}

static void draw_shaded_text(
    Batch2D& batch, const Font& font, const std::wstring& text, int x, int y
) {
    batch.setColor({0, 0, 0, 1.0f});
    font.draw(batch, text, x + 1, y + 1, nullptr, 0);
    batch.resetColor();
    font.draw(batch, text, x, y, nullptr, 0);
}

void SlotView::drawItemInfo(
    Batch2D& batch,
    const ItemStack& stack,
    const ItemDef& item,
    const Font& font,
    const glm::vec2& pos
) {
    const int SLOT_SIZE = InventoryView::SLOT_SIZE;
    if (stack.getCount() > 1) {
        const auto& countStr = cache.countStr;
        int x = pos.x + SLOT_SIZE - countStr.length() * 8;
        int y = pos.y + SLOT_SIZE - 16;
        draw_shaded_text(batch, font, countStr, x, y);
    }

    auto usesPtr = stack.getField("uses");
    if (usesPtr == nullptr || !usesPtr->isInteger()) {
        return;
    }
    int16_t uses = usesPtr->asInteger();
    if (uses < 0) {
        return;
    }
    switch (item.usesDisplay) {
        case ItemUsesDisplay::NONE:
            break;
        case ItemUsesDisplay::RELATION:
            draw_shaded_text(
                batch, font, std::to_wstring(item.uses), pos.x - 3, pos.y + 9
            );
            [[fallthrough]];
        case ItemUsesDisplay::NUMBER:
            draw_shaded_text(
                batch, font, std::to_wstring(uses), pos.x - 3, pos.y - 3
            );
            break;
        case ItemUsesDisplay::VBAR: {
            batch.untexture();
            batch.setColor({0, 0, 0, 0.75f});
            batch.rect(pos.x - 2, pos.y - 2, 6, SLOT_SIZE + 4);
            float t = static_cast<float>(uses) / item.uses;

            int height = SLOT_SIZE * t;
            batch.setColor({(1.0f - t * 0.8f), 0.4f, t * 0.8f + 0.2f, 1.0f});
            batch.rect(pos.x, pos.y + SLOT_SIZE - height, 2, height);
            break;
        }
    }
}

void SlotView::setHighlighted(bool flag) {
    highlighted = flag;
}

bool SlotView::isHighlighted() const {
    return highlighted;
}

void SlotView::performLeftClick(ItemStack& stack, ItemStack& grabbed) {
    const auto& input = gui.getInput();
    if (layout.taking && input.pressed(Keycode::LEFT_SHIFT)) {
        if (layout.shareFunc) {
            layout.shareFunc(layout.index, stack);
        }
        if (layout.updateFunc) {
            layout.updateFunc(layout.index, stack);
        }
        return;
    }
    if (!layout.itemSource && stack.accepts(grabbed) && layout.placing) {
        stack.move(grabbed, *content->getIndices());
    } else {
        if (layout.itemSource) {
            if (grabbed.isEmpty()) {
                grabbed.set(stack);
            } else {
                grabbed.clear();
            }
        } else if (grabbed.isEmpty()) {
            if (layout.taking) {
                std::swap(grabbed, stack);
            }
        } else if (layout.taking && layout.placing) {
            std::swap(grabbed, stack);
        }
    }
}

void SlotView::performRightClick(ItemStack& stack, ItemStack& grabbed) {
    if (layout.rightClick) {
        layout.rightClick(inventoryid, stack);
        if (layout.updateFunc) {
            layout.updateFunc(layout.index, stack);
        }
        return;
    }
    if (layout.itemSource) return;
    if (grabbed.isEmpty()) {
        if (!stack.isEmpty() && layout.taking) {
            grabbed.set(std::move(stack));
            int halfremain = stack.getCount() / 2;
            grabbed.setCount(stack.getCount() - halfremain);
            // reset all data in the origin slot
            stack = ItemStack(stack.getItemId(), halfremain);
        }
        return;
    }
    auto& stackDef = content->getIndices()->items.require(stack.getItemId());
    if (!layout.placing) {
        return;
    }
    if (stack.isEmpty()) {
        itemcount_t count = grabbed.getCount();
        stack.set(std::move(grabbed));
        stack.setCount(1);
        if (count == 1) {
            grabbed = {};
        } else {
            grabbed = ItemStack(stack.getItemId(), count - 1);
        }
    } else if (stack.accepts(grabbed) &&
               stack.getCount() < stackDef.stackSize) {
        stack.setCount(stack.getCount() + 1);
        grabbed.setCount(grabbed.getCount() - 1);
    }
}

void SlotView::clicked(Mousecode button) {
    if (bound == nullptr) return;
    auto exchangeSlot =
        std::dynamic_pointer_cast<SlotView>(gui.get(EXCHANGE_SLOT_NAME));
    if (exchangeSlot == nullptr) {
        return;
    }
    ItemStack& grabbed = exchangeSlot->getStack();
    ItemStack& stack = *bound;

    if (button == Mousecode::BUTTON_1) {
        performLeftClick(stack, grabbed);
    } else if (button == Mousecode::BUTTON_2) {
        performRightClick(stack, grabbed);
    }
    if (layout.updateFunc) {
        layout.updateFunc(layout.index, stack);
    }
}

void SlotView::onFocus() {
    clicked(Mousecode::BUTTON_1);
}

const std::wstring& SlotView::getTooltip() const {
    const auto& str = UINode::getTooltip();
    if (!str.empty() || tooltip.empty()) {
        return str;
    }
    return tooltip;
}

void SlotView::bind(
    int64_t inventoryid, ItemStack& stack, const Content* content
) {
    this->inventoryid = inventoryid;
    bound = &stack;
    this->content = content;
}

const SlotLayout& SlotView::getLayout() const {
    return layout;
}

ItemStack& SlotView::getStack() {
    return *bound;
}

InventoryView::InventoryView(GUI& gui) : Container(gui, glm::vec2()) {
    setColor(glm::vec4(0, 0, 0, 0.0f));
}

InventoryView::~InventoryView() {
}

std::shared_ptr<SlotView> InventoryView::addSlot(const SlotLayout& layout) {
    uint width = InventoryView::SLOT_SIZE + layout.padding;
    uint height = InventoryView::SLOT_SIZE + layout.padding;

    auto pos = layout.position;
    auto vsize = getSize();
    if (pos.x + width > vsize.x) {
        vsize.x = pos.x + width;
    }
    if (pos.y + height > vsize.y) {
        vsize.y = pos.y + height;
    }
    setSize(vsize);

    auto slot = std::make_shared<SlotView>(gui, layout);
    if (!layout.background) {
        slot->setColor(glm::vec4());
    }
    slots.push_back(slot.get());
    return slot;
}

std::shared_ptr<Inventory> InventoryView::getInventory() const {
    return inventory;
}

size_t InventoryView::getSlotsCount() const {
    return slots.size();
}

void InventoryView::bind(
    const std::shared_ptr<Inventory>& inventory, const Content* content
) {
    this->inventory = inventory;
    this->content = content;
    for (auto slot : slots) {
        slot->bind(
            inventory->getId(),
            inventory->getSlot(slot->getLayout().index),
            content
        );
    }
}

void InventoryView::unbind() {
    inventory = nullptr;
}

void InventoryView::setSelected(int index) {
    for (size_t i = 0; i < slots.size(); i++) {
        slots[i]->setHighlighted(static_cast<int>(i) == index);
    }
}

void InventoryView::setPos(glm::vec2 pos) {
    Container::setPos(pos - origin);
}

void InventoryView::setOrigin(glm::vec2 origin) {
    this->origin = origin;
}

glm::vec2 InventoryView::getOrigin() const {
    return origin;
}
