#include "ProgressBar.hpp"

#include <algorithm>
#include <utility>
#include <cmath>

#include "graphics/core/Batch2D.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/Font.hpp"
#include "assets/Assets.hpp"
#include "maths/voxmaths.hpp"
#include "util/stringutil.hpp"

using namespace gui;

ProgressBar::ProgressBar(
    GUI& gui,
    double min,
    double max,
    double value,
    int barThickness
) : UINode(gui, glm::vec2(100, 16)),
    min(min),
    max(max),
    value(value),
    displayValue(value),
    barThickness(barThickness)
{
    if (barThickness <= 0) barThickness = static_cast<int>(size.y);
    setColor(glm::vec4());
    setInteractive(false);
}

void ProgressBar::act(float delta) {
    if (supplier) value = supplier();
    if (smoothTransition) {
        double diff = value - displayValue;
        if (std::fabs(diff) < 2.0) {
            displayValue = value;
        } else {
            double step = smoothSpeed * delta;
            if (std::fabs(diff) < step) {
                displayValue = value;
            } else {
                displayValue += std::copysign(step, diff);
            }
        }
    } else {
        displayValue = value;
    }
}

void ProgressBar::draw(const DrawContext& pctx, const Assets& assets) {
    glm::vec2 pos = calcPos();
    auto batch = pctx.getBatch2D();
    batch->texture(nullptr);

    batch->setColor(bgColor);
    batch->rect(pos.x, pos.y, size.x, size.y);

    float t = 0.0f;
    if (max > min) {
        t = glm::clamp(
            static_cast<float>((displayValue - min) / (max - min)), 0.0f, 1.0f
        );
    }

    if (t > 0.0f) {
        batch->setColor(barColor);
        if (orientation == Orientation::HORIZONTAL) {
            batch->rect(pos.x, pos.y, size.x * t, size.y);
        } else {
            batch->rect(pos.x, pos.y + size.y * (1.0f - t), size.x, size.y * t);
        }
    }

    if (textVisible && !textFormat.empty()) {
        auto font = assets.get<Font>("normal");
        if (font) {
            double pct = (max > min) ? (displayValue - min) / (max - min) * 100.0 : 0.0;
            char buf[64];
            snprintf(buf, sizeof(buf), textFormat.c_str(), displayValue, pct);
            auto wtext = util::str2wstr_utf8(buf);
            int textWidth = font->calcWidth(wtext);
            int lineHeight = font->getLineHeight();
            int tx = static_cast<int>(pos.x + (size.x - textWidth) * 0.5f);
            int ty = static_cast<int>(pos.y + (size.y - lineHeight) * 0.5f);
            batch->setColor(textColor);
            font->draw(*batch, wtext, tx, ty, nullptr, 0);
        }
    }
}

void ProgressBar::setSupplier(doublesupplier supplier) {
    this->supplier = std::move(supplier);
}

double ProgressBar::getValue() const {
    return value;
}

double ProgressBar::getMin() const {
    return min;
}

double ProgressBar::getMax() const {
    return max;
}

double ProgressBar::getProgress() const {
    return (max - min != 0.0) ? (value - min) / (max - min) : 0.0;
}

glm::vec4 ProgressBar::getBarColor() const {
    return barColor;
}

glm::vec4 ProgressBar::getBgColor() const {
    return bgColor;
}

glm::vec4 ProgressBar::getTextColor() const {
    return textColor;
}

const std::string& ProgressBar::getTextFormat() const {
    return textFormat;
}

bool ProgressBar::isSmooth() const {
    return smoothTransition;
}

Orientation ProgressBar::getOrientation() const {
    return orientation;
}

void ProgressBar::setValue(double x) {
    value = x;
}

void ProgressBar::setMin(double x) {
    min = x;
}

void ProgressBar::setMax(double x) {
    max = x;
}

void ProgressBar::setBarThickness(int thickness) {
    barThickness = thickness;
}

void ProgressBar::setBarColor(glm::vec4 color) {
    barColor = color;
}

void ProgressBar::setBgColor(glm::vec4 color) {
    bgColor = color;
}

void ProgressBar::setTextColor(glm::vec4 color) {
    textColor = color;
}

void ProgressBar::setTextFormat(const std::string& fmt) {
    textFormat = fmt;
    textVisible = !fmt.empty();
}

void ProgressBar::setSmooth(bool flag) {
    smoothTransition = flag;
}

void ProgressBar::setOrientation(Orientation ori) {
    orientation = ori;
}
