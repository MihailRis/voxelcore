#include "Label.hpp"

#include <utility>

#include "assets/Assets.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/Batch2D.hpp"
#include "graphics/core/Font.hpp"
#include "graphics/ui/markdown.hpp"
#include "util/stringutil.hpp"

using namespace gui;

void LabelCache::prepare(std::ptrdiff_t fontId, FontMetrics metrics, size_t wrapWidth) {
    if (fontId != this->fontId) {
        resetFlag = true;
        this->fontId = fontId;
        this->metrics = metrics;
    }
    if (wrapWidth != this->wrapWidth) {
        resetFlag = true;
        this->wrapWidth = wrapWidth;
    }
}

size_t LabelCache::getTextLineOffset(size_t line) const {
    line = std::min(lines.size()-1, line);
    return lines.at(line).offset;
}

uint LabelCache::getLineByTextIndex(size_t index) const {
    for (size_t i = 0; i < lines.size(); i++) {
        if (lines[i].offset > index) {
            return i-1;
        }
    }
    return lines.size()-1;
}

void LabelCache::update(std::wstring_view text, bool multiline, bool wrap) {
    resetFlag = false;
    lines.clear();
    lines.push_back(LineScheme {0, false});

    if (fontId == 0) {
        wrap = false;
    }
    
    if (multiline) {
        size_t len = 0;
        for (size_t i = 0; i < text.length(); i++, len++) {
            if (text[i] == L'\n') {
                lines.push_back(LineScheme {i+1, false});
                len = 0;
            } else if (i > 0 && i+1 < text.length() && wrap && text[i+1] != L'\n') {
                size_t width = metrics.calcWidth(text, i-len-1, i-(i-len)+2);
                if (width >= wrapWidth) {
                    // starting a fake line
                    lines.push_back(LineScheme {i+1, true});
                    len = 0;
                }
            }
        }
        if (fontId != 0) {
            int maxWidth = 0;
            for (int i = 0; i < lines.size() - 1; i++) {
                const auto& next = lines[i + 1];
                const auto& cur = lines[i];
                maxWidth = std::max(
                    metrics.calcWidth(
                        text.substr(cur.offset, next.offset - cur.offset)
                    ),
                    maxWidth
                );
            }
            maxWidth = std::max(
                metrics.calcWidth(
                    text.substr(lines[lines.size() - 1].offset)
                ),
                maxWidth
            );
            multilineWidth = maxWidth;
        }
    }
}

Label::Label(GUI& gui, const std::string& text, std::string fontName)
  : UINode(gui, glm::vec2(text.length() * 8, 16)),
    text(util::str2wstr_utf8(text)), 
    fontName(std::move(fontName))
{
    setInteractive(false);
    cache.update(this->text, multiline, textWrap);
}


Label::Label(GUI& gui, const std::wstring& text, std::string fontName)
  : UINode(gui, glm::vec2(text.length() * 8, 16)), 
    text(text), 
    fontName(std::move(fontName))
{
    setInteractive(false);
    cache.update(this->text, multiline, textWrap);
}

Label::~Label() = default;

glm::vec2 Label::calcSize() {
    auto& metrics = cache.metrics;
    uint lineHeight = metrics.lineHeight;
    if (cache.lines.size() > 1) {
        lineHeight *= lineInterval;
    }
    auto view = std::wstring_view(text);
    if (multiline) {
        return glm::vec2(
            cache.multilineWidth,
            lineHeight * cache.lines.size() + metrics.yoffset
        );
    }
    return glm::vec2 (
        cache.metrics.calcWidth(view), 
        lineHeight * cache.lines.size() + metrics.yoffset
    );
}

void Label::setText(std::wstring text) {
    if (markup == "md") {
        auto [processedText, styles] = markdown::process(text, true);
        text = std::move(processedText);
        setStyles(std::move(styles));
    }
    if (text == this->text && !cache.resetFlag) {
        return;
    }
    this->text = std::move(text);
    cache.update(this->text, multiline, textWrap);

    if (cache.fontId != 0 && autoresize) {
        setSize(calcSize());
    }
}

const std::wstring& Label::getText() const {
    return text;
}

void Label::setFontName(std::string name) {
    this->fontName = std::move(name);
}

const std::string& Label::getFontName() const {
    return fontName;
}

void Label::setVerticalAlign(Align align) {
    this->valign = align;
}

Align Label::getVerticalAlign() const {
    return valign;
}

float Label::getLineInterval() const {
    return lineInterval;
}

void Label::setLineInterval(float interval) {
    lineInterval = interval;
}

int Label::getTextYOffset() const {
    return textYOffset;
}

size_t Label::getTextLineOffset(size_t line) const {
    return cache.getTextLineOffset(line);
}

bool Label::isFakeLine(size_t line) const {
    line = std::min(cache.lines.size()-1, line);
    return cache.lines.at(line).fake;
}

int Label::getLineYOffset(uint line) const {
    return line * totalLineHeight + textYOffset;
}

uint Label::getLineByYOffset(int offset) const {
    if (offset < textYOffset) {
        return 0;
    }
    return (offset - textYOffset) / totalLineHeight;
}

uint Label::getLineByTextIndex(size_t index) const {
    return cache.getLineByTextIndex(index);
}

uint Label::getLinesNumber() const {
    return cache.lines.size();
}

void Label::draw(const DrawContext& pctx, const Assets& assets) {
    auto batch = pctx.getBatch2D();
    auto font = assets.get<Font>(fontName);
    cache.prepare(
        reinterpret_cast<ptrdiff_t>(font),
        font->getMetrics(),
        static_cast<size_t>(glm::abs(getSize().x))
    );

    if (supplier) {
        setText(supplier());
    }
    if (cache.resetFlag) {
        cache.update(text, multiline, textWrap);
    }
    batch->setColor(calcColor());

    uint lineHeight = font->getLineHeight();
    if (cache.lines.size() > 1) {
        lineHeight *= lineInterval;
    }
    glm::vec2 size = getSize();
    glm::vec2 newsize = calcSize();
    if (autoresize) {
        setSize(newsize);
    }

    glm::vec2 pos = calcPos();
    switch (align) {
        case Align::left: break;
        case Align::center: pos.x += (size.x-newsize.x)*0.5f; break;
        case Align::right: pos.x += size.x-newsize.x; break;
    }
    switch (valign) {
        case Align::top: break;
        case Align::center: pos.y += (size.y-newsize.y)*0.5f; break;
        case Align::bottom: pos.y += size.y-newsize.y; break;
    }
    textYOffset = pos.y-calcPos().y;
    totalLineHeight = lineHeight;

    const auto& viewport = pctx.getViewport();
    glm::vec4 bounds {0, 0, viewport.x, viewport.y};
    if (parent) {
        auto ppos = parent->calcPos();
        auto psize = parent->getSize();
        bounds.x = std::max(bounds.x, ppos.x);
        bounds.y = std::max(bounds.y, ppos.y);
        bounds.z = std::min(bounds.z, ppos.x + psize.x);
        bounds.w = std::min(bounds.w, ppos.y + psize.y);
    }
    if (multiline) {
        // todo: reduce loop range to the actual area
        for (size_t i = 0; i < cache.lines.size(); i++) {
            float y = pos.y + i * totalLineHeight;
            if (y + totalLineHeight < bounds.y || y > bounds.w) {
                continue;
            }
            auto& line = cache.lines[i];
            size_t offset = line.offset;
            std::wstring_view view(text.c_str()+offset, text.length()-offset);
            if (i < cache.lines.size()-1) {
                view = std::wstring_view(text.c_str()+offset, cache.lines.at(i+1).offset-offset);
            }
            font->draw(*batch, view, pos.x, y, styles.get(), offset);
        }
    } else {
        font->draw(*batch, text, pos.x, pos.y, styles.get(), 0);
    }
}

void Label::textSupplier(wstringsupplier supplier) {
    this->supplier = std::move(supplier);
}

void Label::setAutoResize(bool flag) {
    this->autoresize = flag;
}

bool Label::isAutoResize() const {
    return autoresize;
}

void Label::setMultiline(bool multiline) {
    if (multiline != this->multiline) {
        this->multiline = multiline;
        cache.resetFlag = true;
    }
}

bool Label::isMultiline() const {
    return multiline;
}

void Label::setTextWrapping(bool flag) {
    this->textWrap = flag;
    cache.resetFlag = true;
}

bool Label::isTextWrapping() const {
    return textWrap;
}

void Label::setMarkup(std::string_view lang) {
    markup = lang;
    setText(text);
}

const std::string& Label::getMarkup() const {
    return markup;
}

void Label::setStyles(std::unique_ptr<FontStylesScheme> styles) {
    this->styles = std::move(styles);
}
