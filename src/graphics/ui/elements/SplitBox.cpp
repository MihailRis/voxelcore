#include "SplitBox.hpp"

#include "util/stack_vector.hpp"

using namespace gui;

SplitBox::SplitBox(GUI& gui, const glm::vec2& size, float splitPos, Orientation orientation)
 : BasePanel(gui, size, glm::vec4(), 4.0f, orientation), splitPos(splitPos) {
    setCursor(
        orientation == Orientation::vertical ? CursorShape::NS_RESIZE
                                             : CursorShape::EW_RESIZE
    );
}

void SplitBox::mouseMove(int x, int y) {
    auto pos = calcPos();
    auto size = getSize();
    
    glm::ivec2 cursor(x - pos.x, y - pos.y);
    int axis = orientation == Orientation::vertical;

    int v = cursor[axis];
    v = std::max(std::min(static_cast<int>(size[axis]) - 10, v), 10);
    float t = v / size[axis];
    splitPos = t;

    refresh();
}

void SplitBox::refresh() {
    Container::refresh();

    util::stack_vector<UINode*, 2> visibleNodes;

    for (const auto& node : nodes) {
        if (!node->isVisible()) {
            continue;
        }
        visibleNodes.push_back(node.get());
        if (visibleNodes.full()) {
            break;
        }
    }

    if (visibleNodes.empty()) {
        return;
    }

    glm::vec2 size = getSize();
    if (visibleNodes.size() == 1) {
        auto node = visibleNodes.at(0);
        node->setPos(glm::vec2());
        node->setSize(size);
        return;
    }
    auto nodeA = visibleNodes.at(0);
    auto nodeB = visibleNodes.at(1);

    float sepRadius = interval / 2.0f;
    
    nodeA->setPos(glm::vec2(padding));

    const auto& p = padding;
    if (orientation == Orientation::vertical) {
        float splitPos = this->splitPos * size.y;
        nodeA->setSize({size.x-p.x-p.z, splitPos - sepRadius - p.y});
        nodeB->setSize({size.x-p.x-p.z, size.y - splitPos - sepRadius - p.w});
        nodeB->setPos({p.x, splitPos + sepRadius});
    } else {
        float splitPos = this->splitPos * size.x;
        nodeA->setSize({splitPos - sepRadius - p.x, size.y - p.y - p.w});
        nodeB->setSize({size.x - splitPos - sepRadius - p.z, size.y - p.y - p.w});
        nodeB->setPos({splitPos + sepRadius, p.y});
    }
}

void SplitBox::doubleClick(int x, int y) {
    if (nodes.size() < 2) {
        return;
    }
    std::swap(nodes[0], nodes[1]);
    refresh();
}

void SplitBox::fullRefresh() {
    refresh();
    reposition();
    Container::fullRefresh();
}
