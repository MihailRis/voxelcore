#pragma once

#include "UINode.hpp"
#include "Container.hpp"
#include "typedefs.hpp"
#include "constants.hpp"
#include "items/ItemStack.hpp"

#include <vector>
#include <functional>
#include <glm/glm.hpp>

class Font;
class Assets;
struct ItemDef;
class Batch2D;
class DrawContext;
class Content;
class ContentIndices;
class LevelFrontend;
class Inventory;

namespace gui {
    class UiXmlReader;
}

namespace gui {
    using slotcallback = std::function<void(uint, ItemStack&)>;

    struct SlotLayout {
        int index;
        glm::vec2 position;
        bool background;
        bool itemSource;
        slotcallback updateFunc;
        slotcallback shareFunc;
        slotcallback rightClick;
        int padding = 0;

        bool taking = true;
        bool placing = true;

        SlotLayout(
            int index,
            glm::vec2 position, 
            bool background,
            bool itemSource,
            slotcallback updateFunc,
            slotcallback shareFunc,
            slotcallback rightClick
        );
    };

    class SlotView : public gui::UINode {
        struct {
            ItemStack stack {};
            std::wstring countStr;
        } cache;
    
        const Content* content = nullptr;
        SlotLayout layout;
        bool highlighted = false;

        int64_t inventoryid = 0;
        ItemStack* bound = nullptr;

        void performLeftClick(ItemStack& stack, ItemStack& grabbed);
        void performRightClick(ItemStack& stack, ItemStack& grabbed);

        void drawItemIcon(
            Batch2D& batch,
            const ItemStack& stack,
            const ItemDef& item,
            const Assets& assets,
            const glm::vec4& tint,
            const glm::vec2& pos
        );

        void drawItemInfo(
            Batch2D& batch,
            const ItemStack& stack,
            const ItemDef& item,
            const Font& font,
            const glm::vec2& pos
        );

        void refreshTooltip(const ItemStack& stack, const ItemDef& item);
    public:
        SlotView(GUI& gui, SlotLayout layout);

        virtual void draw(const DrawContext& pctx, const Assets& assets) override;

        void setHighlighted(bool flag);
        bool isHighlighted() const;

        virtual void clicked(Mousecode) override;
        virtual void onFocus() override;
        virtual const std::wstring& getTooltip() const override;

        void bind(
            int64_t inventoryid,
            ItemStack& stack,
            const Content* content
        );

        ItemStack& getStack();
        const SlotLayout& getLayout() const;

        static inline std::string EXCHANGE_SLOT_NAME = "exchange-slot";
    };

    class InventoryView final : public gui::Container {
        const Content* content = nullptr;
        
        std::shared_ptr<Inventory> inventory;

        std::vector<SlotView*> slots;
        glm::vec2 origin {};
    public:
        InventoryView(GUI& gui);
        virtual ~InventoryView();

        void setPos(const glm::vec2& pos) override;

        void setOrigin(const glm::vec2& origin);
        const glm::vec2& getOrigin() const;

        void setSelected(int index);

        void bind(
            const std::shared_ptr<Inventory>& inventory,
            const Content* content
        );
        
        void unbind();

        std::shared_ptr<SlotView> addSlot(const SlotLayout& layout);

        std::shared_ptr<Inventory> getInventory() const;

        size_t getSlotsCount() const;

        static const int SLOT_INTERVAL = 4;
        static const int SLOT_SIZE = ITEM_ICON_SIZE;
    };

    class InventoryBuilder {
        GUI& gui;
        std::shared_ptr<InventoryView> view;
    public:
        InventoryBuilder(GUI& gui);

        /// @brief Add slots grid to inventory view 
        /// @param cols grid columns
        /// @param count total number of grid slots
        /// @param pos position of the first slot of the grid
        /// @param padding additional space around the grid
        /// @param addpanel automatically create panel behind the grid
        /// with size including padding
        /// @param slotLayout slot settings (index and position are ignored)
        void addGrid(
            int cols, int count, 
            glm::vec2 pos, 
            glm::vec4 padding,
            bool addpanel,
            const SlotLayout& slotLayout
        );
        
        void add(const SlotLayout& slotLayout);
        std::shared_ptr<InventoryView> build();
    };
}
