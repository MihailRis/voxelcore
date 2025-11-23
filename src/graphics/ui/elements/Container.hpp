#pragma once

#include "UINode.hpp"
#include "commons.hpp"
#include "util/ObjectsKeeper.hpp"

#include <vector>

namespace gui {
    class Container : public UINode, public ::util::ObjectsKeeper {
        int prevScrollY = -1;
    protected:
        std::vector<std::shared_ptr<UINode>> nodes;
        std::vector<IntervalEvent> intervalEvents;
        int scroll = 0;
        int scrollStep = 40;
        int scrollBarWidth = 10;
        int actualLength = 0;
        bool scrollable = true;

        bool isScrolling() {
            return prevScrollY != -1;
        }
    public:
        Container(GUI& gui, glm::vec2 size);
        virtual ~Container();

        virtual void act(float delta) override;
        virtual void drawBackground(const DrawContext& pctx, const Assets& assets);
        virtual void draw(const DrawContext& pctx, const Assets& assets) override;
        virtual std::shared_ptr<UINode> getAt(const glm::vec2& pos) override;
        virtual void add(const std::shared_ptr<UINode>& node);
        virtual void add(const std::shared_ptr<UINode>& node, glm::vec2 pos);
        virtual void clear();
        virtual void remove(UINode* node);
        virtual void remove(const std::string& id);
        virtual void scrolled(int value) override;
        virtual void setScrollable(bool flag);
        void listenInterval(float interval, OnTimeOut callback, int repeat=-1);
        virtual glm::vec2 getContentOffset() const override {
            return glm::vec2(0.0f, scroll);
        };
        virtual void setSize(const glm::vec2& size) override;
        virtual int getScrollStep() const;
        virtual void setScrollStep(int step);
        virtual void refresh() override;
        void setScroll(int scroll);

        virtual void mouseMove(int x, int y) override;
        virtual void mouseRelease(int x, int y) override;

        const std::vector<std::shared_ptr<UINode>>& getNodes() const;
    };
}
