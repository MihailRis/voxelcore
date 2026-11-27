#include "ProgressBar.hpp"

#include <cmath>
#include <utility>

#include "graphics/core/Batch2D.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/Font.hpp"
#include "assets/Assets.hpp"

using namespace gui;

ProgressBar::ProgressBar(
    GUI& gui,
    double min,
    double max,
    double value
) : UINode(gui, glm::vec2(100, 16)),
    min(min),
    max(max),
    value(value),
    displayValue(value)
{
    setColor(glm::vec4(0.2f, 0.6f, 1.0f, 0.8f));
    setInteractive(false);
}

void ProgressBar::act(float delta) {
    UINode::act(delta);
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
            static_cast<float>((displayValue - min) / (max - min)),
            0.0f,
            1.0f
        );
    }

    if (t > 0.0f) {
        batch->setColor(color);
        if (orientation == Orientation::HORIZONTAL) {
            batch->rect(pos.x, pos.y, size.x * t, size.y);
        } else {
            batch->rect(pos.x, pos.y + size.y * (1.0f - t), size.x, size.y * t);
        }
    }

    if (!text.empty()) {
        auto font = assets.getShared<Font>(fontName);
        if (font) {
            int textWidth = font->calcWidth(text);
            int lineHeight = font->getLineHeight();
            int tx = static_cast<int>(pos.x + (size.x - textWidth) * 0.5f);
            int ty = static_cast<int>(pos.y + (size.y - lineHeight) * 0.5f);
            batch->setColor(textColor);
            font->draw(*batch, text, tx, ty, nullptr, 0);
        }
    }
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

double ProgressBar::getDisplayValue() const {
    return displayValue;
}

glm::vec4 ProgressBar::getBgColor() const {
    return bgColor;
}

glm::vec4 ProgressBar::getTextColor() const {
    return textColor;
}

const std::wstring& ProgressBar::getText() const {
    return text;
}

bool ProgressBar::isSmooth() const {
    return smoothTransition;
}

float ProgressBar::getSmoothSpeed() const {
    return smoothSpeed;
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

void ProgressBar::setBgColor(glm::vec4 color) {
    bgColor = color;
}

void ProgressBar::setTextColor(glm::vec4 color) {
    textColor = color;
}

void ProgressBar::setText(std::wstring text) {
    this->text = std::move(text);
}

void ProgressBar::setFontName(std::string name) {
    fontName = std::move(name);
}

const std::string& ProgressBar::getFontName() const {
    return fontName;
}

void ProgressBar::setSmooth(bool flag) {
    smoothTransition = flag;
}

void ProgressBar::setSmoothSpeed(float speed) {
    smoothSpeed = speed;
}

void ProgressBar::setOrientation(Orientation ori) {
    orientation = ori;
}
