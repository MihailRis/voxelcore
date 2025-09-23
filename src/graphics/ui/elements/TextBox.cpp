#include "TextBox.hpp"

#include <algorithm>
#include <sstream>
#include <utility>

#include "../GUI.hpp"
#include "../markdown.hpp"
#include "Label.hpp"
#include "assets/Assets.hpp"
#include "devtools/Editor.hpp"
#include "devtools/SyntaxProcessor.hpp"
#include "engine/Engine.hpp"
#include "graphics/core/Batch2D.hpp"
#include "graphics/core/DrawContext.hpp"
#include "graphics/core/Font.hpp"
#include "graphics/ui/markdown.hpp"
#include "util/stringutil.hpp"
#include "window/Window.hpp"
#include "devtools/actions.hpp"
#include "../markdown.hpp"

using namespace gui;

inline constexpr int LINE_NUMBERS_PANE_WIDTH = 40;

class InputAction : public Action {
    std::weak_ptr<TextBox> textbox;
    size_t position;
    std::wstring string;
public:
    InputAction(
        std::weak_ptr<TextBox> textbox, size_t position, std::wstring string
    )
        : textbox(std::move(textbox)),
          position(position),
          string(std::move(string)) {
    }
    void apply() override {
        if (auto box = textbox.lock()) {
            box->select(position, position);
            box->paste(string);
        }
    }

    void revert() override {
        if (auto box = textbox.lock()) {
            box->select(position, position);
            box->erase(position, string.length());
        }
    }
};

class SelectionAction : public Action {
    std::weak_ptr<TextBox> textbox;
    size_t start;
    size_t end;
public:
    SelectionAction(std::weak_ptr<TextBox> textbox, size_t start, size_t end)
    : textbox(std::move(textbox)), start(start), end(end) {}

    void apply() override {
        if (auto box = textbox.lock()) {
            box->select(start, end);
        }
    }

    void revert() override {
        if (auto box = textbox.lock()) {
            box->select(0, 0);
        }
    }
};

namespace gui {
    /// @brief Accumulates small changes into words for InputAction creation
    class TextBoxHistorian {
    public:
        TextBoxHistorian(TextBox& textBox, ActionsHistory& history)
            : textBox(textBox), history(history) {
        }

        void onPaste(size_t pos, std::wstring_view text) {
            if (locked) {
                return;
            }
            if (erasing) {
                sync();
            }
            if (this->pos == static_cast<size_t>(-1)) {
                this->pos = pos;
            }
            if (this->pos + length != pos || text == L" " || text == L"\n") {
                sync();
                this->pos = pos;
            }
            ss << text;
            length += text.length();
        }

        void onErase(size_t pos, std::wstring_view text, bool selection=false) {
            if (locked) {
                return;
            }
            if (!erasing) {
                sync();
                erasing = true;
            }
            if (selection) {
                history.store(
                    std::make_unique<SelectionAction>(
                        getTextBoxWeakptr(),
                        textBox.getSelectionStart(),
                        textBox.getSelectionEnd()
                    ),
                    true
                );
            }
            if (this->pos == static_cast<size_t>(-1)) {
                this->pos = pos;
            } else if (this->pos - text.length() != pos) {
                sync();
                erasing = true;
                this->pos = pos;
            }
            if (text == L" " || text == L"\n") {
                sync();
                erasing = true;
                this->pos = pos;
            }
            auto str = ss.str();
            ss.seekp(0);
            ss << text << str;

            this->pos = pos;
            length += text.length();
        }

        /// @brief Flush buffer and push all changes to the ActionsHistory
        void sync() {
            auto string = ss.str();
            if (string.empty()) {
                return;
            }
            auto action =
                std::make_unique<InputAction>(getTextBoxWeakptr(), pos, string);
            history.store(std::move(action), erasing);
            reset();
        }

        void undo() {
            sync();
            locked = true;
            history.undo();
            locked = false;
        }

        void redo() {
            sync();
            locked = true;
            history.redo();
            locked = false;
        }

        void reset() {
            pos = -1;
            length = 0;
            erasing = false;
            ss = {};
        }

        bool isSynced() const {
            return length == 0;
        }
    private:
        TextBox& textBox;
        ActionsHistory& history;
        std::wstringstream ss;
        size_t pos = -1;
        size_t length = 0;
        bool erasing = false;
        bool locked = false;

        std::weak_ptr<TextBox> getTextBoxWeakptr() {
            return std::weak_ptr<TextBox>(std::dynamic_pointer_cast<TextBox>(
                textBox.shared_from_this()
            ));
        }
    };
}

TextBox::TextBox(GUI& gui, std::wstring placeholder, glm::vec4 padding)
    : Container(gui, glm::vec2(200, 32)),
      inputEvents(gui.getInput()),
      history(std::make_shared<ActionsHistory>()),
      historian(std::make_unique<TextBoxHistorian>(*this, *history)),
      padding(padding),
      input(L""),
      placeholder(std::move(placeholder)) {
    setCursor(CursorShape::TEXT);
    setOnUpPressed(nullptr);
    setOnDownPressed(nullptr);
    setColor(glm::vec4(0.0f, 0.0f, 0.0f, 0.75f));

    label = std::make_shared<Label>(gui, L"");
    label->setSize(
        size - glm::vec2(padding.z + padding.x, padding.w + padding.y)
    );
    label->setPos(glm::vec2(
        padding.x + LINE_NUMBERS_PANE_WIDTH * showLineNumbers, padding.y
    ));
    add(label);

    lineNumbersLabel = std::make_shared<Label>(gui, L"");
    lineNumbersLabel->setMultiline(true);
    lineNumbersLabel->setSize(
        size - glm::vec2(padding.z + padding.x, padding.w + padding.y)
    );
    lineNumbersLabel->setVerticalAlign(Align::top);
    add(lineNumbersLabel);

    setHoverColor(glm::vec4(0.05f, 0.1f, 0.2f, 0.75f));

    textInitX = label->getPos().x;
    scrollable = true;
    scrollStep = 0;
}

TextBox::~TextBox() = default;

void TextBox::draw(const DrawContext& pctx, const Assets& assets) {
    Container::draw(pctx, assets);

    font = assets.get<Font>(label->getFontName());

    if (!isFocused()) {
        return;
    }
    const auto& labelText = getText();

    glm::vec2 pos = calcPos();
    glm::vec2 size = getSize();

    auto subctx = pctx.sub();
    subctx.setScissors(glm::vec4(pos.x, pos.y, size.x, size.y));

    const int lineHeight = font->getLineHeight() * label->getLineInterval();
    glm::vec2 lcoord = label->calcPos();
    lcoord.y -= 2;
    auto batch = pctx.getBatch2D();
    batch->texture(nullptr);
    batch->setColor(glm::vec4(1.0f));

    float time = gui.getWindow().time();

    if (editable && static_cast<int>((time - caretLastMove) * 2) % 2 == 0) {
        uint line = rawTextCache.getLineByTextIndex(caret);
        uint lcaret = caret - rawTextCache.getTextLineOffset(line);
        int width = font->calcWidth(input, lcaret);
        batch->rect(
            lcoord.x + width,
            lcoord.y + label->getLineYOffset(line),
            2,
            lineHeight
        );
    }
    if (selectionStart != selectionEnd) {
        auto selectionCtx = subctx.sub(batch);
        selectionCtx.setBlendMode(BlendMode::addition);

        uint startLine = label->getLineByTextIndex(selectionStart);
        uint endLine = label->getLineByTextIndex(selectionEnd);

        batch->setColor(glm::vec4(0.8f, 0.9f, 1.0f, 0.25f));
        int start = font->calcWidth(
            labelText, selectionStart - label->getTextLineOffset(startLine)
        );
        int end = font->calcWidth(
            labelText, selectionEnd - label->getTextLineOffset(endLine)
        );
        int lineY = label->getLineYOffset(startLine);

        if (startLine == endLine) {
            batch->rect(
                lcoord.x + start, lcoord.y + lineY, end - start, lineHeight
            );
        } else {
            batch->rect(
                lcoord.x + start,
                lcoord.y + lineY,
                label->getSize().x - start - padding.z - padding.x - 2,
                lineHeight
            );
            for (uint i = startLine + 1; i < endLine; i++) {
                batch->rect(
                    lcoord.x,
                    lcoord.y + label->getLineYOffset(i),
                    label->getSize().x - padding.z - padding.x - 2,
                    lineHeight
                );
            }
            batch->rect(
                lcoord.x,
                lcoord.y + label->getLineYOffset(endLine),
                end,
                lineHeight
            );
        }
    }

    if (isFocused() && multiline) {
        auto selectionCtx = subctx.sub(batch);
        selectionCtx.setBlendMode(BlendMode::addition);

        batch->setColor(glm::vec4(1, 1, 1, 0.1f));

        uint line = label->getLineByTextIndex(caret);
        while (label->isFakeLine(line)) {
            line--;
        }
        do {
            int lineY = label->getLineYOffset(line);

            batch->setColor(glm::vec4(1, 1, 1, 0.05f));
            if (showLineNumbers) {
                batch->rect(
                    lcoord.x - 8,
                    lcoord.y + lineY,
                    label->getSize().x,
                    lineHeight
                );
                batch->setColor(glm::vec4(1, 1, 1, 0.10f));
                batch->rect(
                    lcoord.x - LINE_NUMBERS_PANE_WIDTH,
                    lcoord.y + lineY,
                    LINE_NUMBERS_PANE_WIDTH - 8,
                    lineHeight
                );
            } else {
                batch->rect(
                    lcoord.x, lcoord.y + lineY, label->getSize().x, lineHeight
                );
            }
            line++;
        } while (line < label->getLinesNumber() && label->isFakeLine(line));
    }
}

void TextBox::drawBackground(const DrawContext& pctx, const Assets&) {
    glm::vec2 pos = calcPos();

    auto batch = pctx.getBatch2D();
    batch->texture(nullptr);

    auto subctx = pctx.sub();
    subctx.setScissors(glm::vec4(pos.x, pos.y - 0.5, size.x, size.y + 1));

    if (valid) {
        if (isFocused() && !multiline) {
            batch->setColor(focusedColor);
        } else if (hover && !multiline) {
            batch->setColor(hoverColor);
        } else {
            batch->setColor(color);
        }
    } else {
        batch->setColor(invalidColor);
    }

    batch->rect(pos.x, pos.y, size.x, size.y);
    if (!isFocused() && supplier) {
        input = supplier();
    }
    refreshLabel();
}

void TextBox::refreshLabel() {
    rawTextCache.prepare(font, static_cast<size_t>(getSize().x));
    rawTextCache.update(input, multiline, false);

    label->setColor(textColor * glm::vec4(input.empty() ? 0.5f : 1.0f));

    const auto& displayText = input.empty() && !hint.empty() ? hint : getText();
    if (markup == "md") {
        auto [processedText, styles] =
            markdown::process(displayText, !focused || !editable);
        label->setText(std::move(processedText));
        label->setStyles(std::move(styles));
    } else {
        label->setText(displayText);
        if (syntax.empty()) {
            label->setStyles(nullptr);
        }
    }

    if (showLineNumbers) {
        if (lineNumbersLabel->getLinesNumber() != label->getLinesNumber()) {
            std::wstringstream ss;
            int n = 1;
            for (int i = 1; i <= label->getLinesNumber(); i++) {
                if (!label->isFakeLine(i - 1)) {
                    ss << n;
                    n++;
                }
                if (i + 1 <= label->getLinesNumber()) {
                    ss << "\n";
                }
            }
            lineNumbersLabel->setText(ss.str());
        }
        lineNumbersLabel->setPos(padding);
        lineNumbersLabel->setColor(glm::vec4(1, 1, 1, 0.25f));
    }

    if (autoresize && font) {
        auto size = getSize();
        int newy = glm::min(
            static_cast<int>(parent->getSize().y),
            static_cast<int>(
                label->getLinesNumber() * label->getLineInterval() *
                font->getLineHeight()
            ) + 1
        );
        if (newy != static_cast<int>(size.y)) {
            size.y = newy;
            setSize(size);
            if (positionfunc) {
                pos = positionfunc();
            }
        }
    }

    if (multiline && font) {
        setScrollable(true);
        uint height = label->getLinesNumber() * font->getLineHeight() *
                      label->getLineInterval();
        label->setSize(glm::vec2(label->getSize().x, height));
        actualLength = height;
    } else {
        setScrollable(false);
    }
}

/// @brief Insert text at the caret. Also selected text will be erased
/// @param text Inserting text
void TextBox::paste(const std::wstring& text, bool history) {
    eraseSelected();
    auto inputText = text;
    inputText.erase(
        std::remove(inputText.begin(), inputText.end(), '\r'), inputText.end()
    );
    historian->onPaste(caret, inputText);
    if (caret >= input.length()) {
        input += inputText;
    } else {
        auto left = input.substr(0, caret);
        auto right = input.substr(caret);
        input = left + inputText + right;
    }
    refreshLabel();
    setCaret(caret + inputText.length());
    if (validate()) {
        onInput();
    }
}

/// @brief Remove part of the text and move caret to start of the part
/// @param start start of the part
/// @param length length of part that will be removed
void TextBox::erase(size_t start, size_t length) {
    size_t end = start + length;
    if (caret > start) {
        setCaret(caret - length);
    }
    auto left = input.substr(0, start);
    auto right = input.substr(end);
    input = left + right;
}

/// @brief Remove all selected text and reset selection
/// @return true if erased anything
bool TextBox::eraseSelected() {
    if (selectionStart == selectionEnd) {
        return false;
    }
    historian->onErase(
        selectionStart,
        input.substr(selectionStart, selectionEnd - selectionStart),
        true
    );
    erase(selectionStart, selectionEnd - selectionStart);
    resetSelection();
    onInput();
    return true;
}

void TextBox::resetSelection() {
    selectionOrigin = 0;
    selectionStart = 0;
    selectionEnd = 0;
}

void TextBox::extendSelection(int index) {
    size_t normalized = normalizeIndex(index);
    selectionStart = std::min(selectionOrigin, normalized);
    selectionEnd = std::max(selectionOrigin, normalized);
}

size_t TextBox::getLineLength(uint line) const {
    size_t position = label->getTextLineOffset(line);
    size_t lineLength = label->getTextLineOffset(line + 1) - position;
    if (lineLength == 0) {
        lineLength = label->getText().length() - position + 1;
    }
    return lineLength;
}

size_t TextBox::getSelectionLength() const {
    return selectionEnd - selectionStart;
}

/// @brief Set scroll offset
/// @param x scroll offset
void TextBox::setTextOffset(uint x) {
    textOffset = x;
    refresh();
}

void TextBox::typed(unsigned int codepoint) {
    if (editable) {
        // Combine deleting selected text and inserting a symbol
        auto combination = history->beginCombination();
        paste(std::wstring({static_cast<wchar_t>(codepoint)}));
    }
}

bool TextBox::validate() {
    if (validator) {
        valid = validator(getText());
    } else {
        valid = true;
    }
    return valid;
}

void TextBox::setValid(bool valid) {
    this->valid = valid;
}

bool TextBox::isValid() const {
    return valid;
}

void TextBox::setMultiline(bool multiline) {
    this->multiline = multiline;
    label->setMultiline(multiline);
    label->setVerticalAlign(multiline ? Align::top : Align::center);
}

bool TextBox::isMultiline() const {
    return multiline;
}

void TextBox::setTextWrapping(bool flag) {
    label->setTextWrapping(flag);
}

bool TextBox::isTextWrapping() const {
    return label->isTextWrapping();
}

void TextBox::setEditable(bool editable) {
    this->editable = editable;
}

bool TextBox::isEditable() const {
    return editable;
}

bool TextBox::isEdited() const {
    return history->size() != editedHistorySize || !historian->isSynced();
}

void TextBox::setUnedited() {
    historian->sync();
    editedHistorySize = history->size();
}

size_t TextBox::getSelectionStart() const {
    return selectionStart;
}

size_t TextBox::getSelectionEnd() const {
    return selectionEnd;
}

void TextBox::setOnEditStart(runnable oneditstart) {
    onEditStart = oneditstart;
}

void TextBox::setAutoResize(bool flag) {
    this->autoresize = flag;
}

bool TextBox::isAutoResize() const {
    return autoresize;
}

void TextBox::onFocus() {
    Container::onFocus();
    if (onEditStart) {
        setCaret(input.size());
        onEditStart();
        resetSelection();
    }
}

void TextBox::reposition() {
    UINode::reposition();
    refreshLabel();
}

void TextBox::refresh() {
    Container::refresh();
    label->setSize(
        size - glm::vec2(padding.z + padding.x, padding.w + padding.y)
    );
    label->setPos(glm::vec2(
        padding.x + LINE_NUMBERS_PANE_WIDTH * showLineNumbers + textInitX -
            static_cast<int>(textOffset),
        padding.y
    ));
}

/// @brief Clamp index to range [0, input.length()]
/// @param index non-normalized index
/// @return normalized index
size_t TextBox::normalizeIndex(int index) {
    return std::min(input.length(), static_cast<size_t>(std::max(0, index)));
}

/// @brief Calculate index of character at defined screen X position
/// @param x screen X position
/// @param y screen Y position
/// @return non-normalized character index
int TextBox::calcIndexAt(int x, int y) const {
    if (font == nullptr) return 0;
    const auto& labelText = label->getText();
    glm::vec2 lcoord = label->calcPos();
    uint line = label->getLineByYOffset(y - lcoord.y);
    line = std::min(line, label->getLinesNumber() - 1);
    size_t lineLength = getLineLength(line);
    uint offset = 0;
    while (lcoord.x + font->calcWidth(labelText, offset) < x &&
           offset < lineLength - 1) {
        offset++;
    }
    return std::min(
        offset + label->getTextLineOffset(line), labelText.length()
    );
}

static inline std::wstring get_alphabet(wchar_t c) {
    std::wstring alphabet {c};
    if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '_') {
        return L"abcdefghijklmnopqrstuvwxyz_"
               L"ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    } else if (c >= '0' && c <= '9') {
        return L"0123456789";
    }
    return alphabet;
}

void TextBox::tokenSelectAt(int index) {
    const auto& actualText = label->getText();
    if (actualText.empty()) {
        return;
    }
    int left = index;
    int right = index;

    std::wstring alphabet = get_alphabet(actualText.at(index));
    while (left >= 0) {
        if (alphabet.find(actualText.at(left)) == std::wstring::npos) {
            break;
        }
        left--;
    }
    while (static_cast<size_t>(right) < actualText.length()) {
        if (alphabet.find(actualText.at(right)) == std::wstring::npos) {
            break;
        }
        right++;
    }
    select(left + 1, right);
}

void TextBox::doubleClick(int x, int y) {
    UINode::doubleClick(x, y);
    tokenSelectAt(normalizeIndex(calcIndexAt(x, y) - 1));
}

void TextBox::click(int x, int y) {
    int index = normalizeIndex(calcIndexAt(x, y));
    selectionStart = index;
    selectionEnd = index;
    selectionOrigin = index;
}

void TextBox::mouseMove(int x, int y) {
    Container::mouseMove(x, y);
    if (isScrolling()) {
        return;
    }
    ptrdiff_t index = calcIndexAt(x, y);
    setCaret(index);
    extendSelection(index);
    resetMaxLocalCaret();
}

void TextBox::resetMaxLocalCaret() {
    maxLocalCaret =
        caret - label->getTextLineOffset(label->getLineByTextIndex(caret));
}

void TextBox::stepLeft(bool shiftPressed, bool breakSelection) {
    uint previousCaret = this->caret;
    size_t caret = breakSelection ? selectionStart : this->caret;
    if (caret > 0) {
        if (caret > input.length()) {
            setCaret(input.length() - 1);
        } else {
            setCaret(caret - 1);
        }
        if (shiftPressed) {
            if (selectionStart == selectionEnd) {
                selectionOrigin = previousCaret;
            }
            extendSelection(this->caret);
        } else {
            resetSelection();
        }
    } else {
        setCaret(caret);
        resetSelection();
    }
    resetMaxLocalCaret();
}

void TextBox::stepRight(bool shiftPressed, bool breakSelection) {
    uint previousCaret = this->caret;
    size_t caret = breakSelection ? selectionEnd : this->caret;
    if (caret < input.length()) {
        setCaret(caret + 1);
        caretLastMove = gui.getWindow().time();
        if (shiftPressed) {
            if (selectionStart == selectionEnd) {
                selectionOrigin = previousCaret;
            }
            extendSelection(this->caret);
        } else {
            resetSelection();
        }
    } else {
        setCaret(caret);
        resetSelection();
    }
    resetMaxLocalCaret();
}

void TextBox::stepDefaultDown(bool shiftPressed, bool breakSelection) {
    uint previousCaret = this->caret;
    uint caret = breakSelection ? selectionEnd : this->caret;
    uint caretLine = label->getLineByTextIndex(caret);
    if (caretLine < label->getLinesNumber() - 1) {
        uint offset =
            std::min(size_t(maxLocalCaret), getLineLength(caretLine + 1) - 1);
        setCaret(label->getTextLineOffset(caretLine + 1) + offset);
    } else {
        setCaret(input.length());
    }
    if (shiftPressed) {
        if (selectionStart == selectionEnd) {
            selectionOrigin = previousCaret;
        }
        extendSelection(this->caret);
    } else {
        resetSelection();
    }
}

void TextBox::stepDefaultUp(bool shiftPressed, bool breakSelection) {
    uint previousCaret = this->caret;
    uint caret = breakSelection ? selectionStart : this->caret;
    uint caretLine = label->getLineByTextIndex(caret);
    if (caretLine > 0) {
        uint offset =
            std::min(size_t(maxLocalCaret), getLineLength(caretLine - 1) - 1);
        setCaret(label->getTextLineOffset(caretLine - 1) + offset);
    } else {
        setCaret(static_cast<size_t>(0));
    }
    if (shiftPressed) {
        if (selectionStart == selectionEnd) {
            selectionOrigin = previousCaret;
        }
        extendSelection(this->caret);
    } else {
        resetSelection();
    }
}

static int calc_indent(int linestart, std::wstring_view input) {
    int indent = 0;
    while (linestart + indent < input.length() &&
           input[linestart + indent] == L' ')
        indent++;
    return indent;
}

void TextBox::onTab(bool shiftPressed) {
    std::wstring indentStr = L"    ";

    if (!shiftPressed && getSelectionLength() == 0) {
        paste(indentStr);
        return;
    }
    if (getSelectionLength() == 0) {
        selectionStart = caret;
        selectionEnd = caret;
        selectionOrigin = caret;
    }

    int lineA = getLineAt(selectionStart);
    int lineB = getLineAt(selectionEnd);
    int caretLine = getLineAt(caret);

    size_t lineAStart = getLinePos(lineA);
    size_t lineBStart = getLinePos(lineB);
    size_t caretLineStart = getLinePos(caretLine);
    size_t caretIndent = calc_indent(caretLineStart, input);
    size_t aIndent = calc_indent(lineAStart, input);
    size_t bIndent = calc_indent(lineBStart, input);

    int lastSelectionStart = selectionStart;
    int lastSelectionEnd = selectionEnd;
    size_t lastCaret = caret;
    
    auto combination = history->beginCombination();

    resetSelection();

    for (int line = lineA; line <= lineB; line++) {
        size_t linestart = getLinePos(line);
        int indent = calc_indent(linestart, input);
        
        if (shiftPressed) {
            if (indent >= indentStr.length()) {
                setCaret(linestart);
                select(linestart, linestart + indentStr.length());
                eraseSelected();
            }
        } else {
            setCaret(linestart);
            paste(indentStr);
        }
        refreshLabel(); // todo: replace with textbox cache
    }

    int linestart = getLinePos(caretLine);
    int linestartA = getLinePos(lineA);
    int linestartB = getLinePos(lineB);
    int la = lastSelectionStart - lineAStart;
    int lb = lastSelectionEnd - lineBStart;
    if (shiftPressed) {
        setCaret(lastCaret - caretLineStart + linestart - std::min<int>(caretIndent, indentStr.length()));
        selectionStart = la + linestartA - std::min<int>(std::min<int>(la, aIndent), indentStr.length());
        selectionEnd = lb + linestartB - std::min<int>(std::min<int>(lb, bIndent), indentStr.length());
    } else {
        setCaret(lastCaret - caretLineStart + linestart + indentStr.length());
        selectionStart = la + linestartA + indentStr.length();
        selectionEnd = lb + linestartB + indentStr.length();
    }
    if (selectionOrigin == lastSelectionStart) {
        selectionOrigin = selectionStart;
    } else {
        selectionOrigin = selectionEnd;
    }
    historian->sync();
}

void TextBox::refreshSyntax() {
    if (!syntax.empty()) {
        const auto& processor = gui.getEditor().getSyntaxProcessor();
        if (auto styles = processor.highlight(syntax, input)) {
            label->setStyles(std::move(styles));
        }
    }
}

void TextBox::onInput() {
    if (subconsumer) {
        subconsumer(input);
    }
    refreshSyntax();
}

void TextBox::performEditingKeyboardEvents(Keycode key) {
    bool shiftPressed = gui.getInput().pressed(Keycode::LEFT_SHIFT);
    bool breakSelection = getSelectionLength() != 0 && !shiftPressed;

    uint current_line = getLineAt(getCaret());
    uint previousCaret = getCaret();
    
    if (key == Keycode::BACKSPACE) {
        bool erased = eraseSelected();
        if (erased) {
            if (validate()) {
                onInput();
            }
        } else if (caret > 0 && input.length() > 0) {
            if (caret > input.length()) {
                caret = input.length();
            }
            historian->onErase(caret - 1, input.substr(caret - 1, 1));
            input = input.substr(0, caret - 1) + input.substr(caret);
            setCaret(caret - 1);
            if (validate()) {
                onInput();
            }
        }
    } else if (key == Keycode::DELETE) {
        if (!eraseSelected() && caret < input.length()) {
            historian->onErase(caret, input.substr(caret, 1));
            input = input.substr(0, caret) + input.substr(caret + 1);
            if (validate()) {
                onInput();
            }
        }
    } else if (key == Keycode::ENTER) {
        if (multiline) {
            paste(L"\n");
        } else {
            defocus();
            if (validate() && consumer) {
                consumer(getText());
            }
        }
    } else if (key == Keycode::TAB) {
        onTab(shiftPressed);
    } else if (key == Keycode::LEFT) {
        stepLeft(shiftPressed, breakSelection);
    } else if (key == Keycode::RIGHT) {
        stepRight(shiftPressed, breakSelection);
    } else if (key == Keycode::UP && onUpPressed) {
        onUpPressed();
    } else if (key == Keycode::DOWN && onDownPressed) {
        onDownPressed();
    } else if (key == Keycode::HOME) {
        setCaret(getLinePos(current_line));
        resetMaxLocalCaret();

        if (shiftPressed) {
            if (selectionStart == selectionEnd) {
                selectionOrigin = previousCaret;
            }
            extendSelection(getCaret());
        } else {
            resetSelection();
        }
    } else if (key == Keycode::END && getLineLength(current_line) > 0) {
        setCaret(getLinePos(current_line) + getLineLength(current_line) - 1);
        resetMaxLocalCaret();
	
        if (shiftPressed) {
            if (selectionStart == selectionEnd) {
                selectionOrigin = previousCaret;
            }
            extendSelection(getCaret());
        } else {
            resetSelection();
        }
    }
}

void TextBox::keyPressed(Keycode key) {
    const auto& inputEvents = gui.getInput();
    if (editable) {
        performEditingKeyboardEvents(key);
    }
    if (inputEvents.pressed(Keycode::LEFT_CONTROL) && key != Keycode::LEFT_CONTROL) {
        if (controlCombinationsHandler) {
            if (controlCombinationsHandler(static_cast<int>(key))) {
                return;
            }
        }
        // Copy selected text to clipboard
        if (key == Keycode::C || key == Keycode::X) {
            std::string text = util::wstr2str_utf8(getSelection());
            if (!text.empty()) {
                gui.getInput().setClipboardText(text.c_str());
            }
            if (editable && key == Keycode::X) {
                eraseSelected();
            }
        }
        // Paste text from clipboard
        if (key == Keycode::V && editable) {
            const char* text = inputEvents.getClipboardText();
            if (text) {
                historian->sync(); // flush buffer before combination
                // Combine deleting selected text and pasing a clipboard content
                auto combination = history->beginCombination();
                paste(util::str2wstr_utf8(text));
                historian->sync();
            }
        }
        // Select/deselect all
        if (key == Keycode::A) {
            if (selectionStart == selectionEnd) {
                select(0, input.length());
            } else {
                resetSelection();
            }
        }
        if (key == Keycode::Z) {
            historian->undo();
            refreshSyntax();
        }
        if (key == Keycode::Y) {
            historian->redo();
            refreshSyntax();
        }
    }
}

void TextBox::select(int start, int end) {
    if (end < start) {
        std::swap(start, end);
    }
    start = normalizeIndex(start);
    end = normalizeIndex(end);

    selectionStart = start;
    selectionEnd = end;
    selectionOrigin = start;
    setCaret(selectionEnd);
}

uint TextBox::getLineAt(size_t position) const {
    return label->getLineByTextIndex(position);
}

size_t TextBox::getLinePos(uint line) const {
    return label->getTextLineOffset(line);
}

std::shared_ptr<UINode> TextBox::getAt(const glm::vec2& pos) {
    return UINode::getAt(pos);
}

void TextBox::setOnUpPressed(const runnable& callback) {
    if (callback == nullptr) {
        onUpPressed = [this]() {
            if (inputEvents.pressed(Keycode::LEFT_CONTROL)) {
                scrolled(1);
                return;
            }
            bool shiftPressed = inputEvents.pressed(Keycode::LEFT_SHIFT);
            bool breakSelection = getSelectionLength() != 0 && !shiftPressed;
            stepDefaultUp(shiftPressed, breakSelection);
        };
    } else {
        onUpPressed = callback;
    }
}

void TextBox::setOnDownPressed(const runnable& callback) {
    if (callback == nullptr) {
        onDownPressed = [this]() {
            if (inputEvents.pressed(Keycode::LEFT_CONTROL)) {
                scrolled(-1);
                return;
            }
            bool shiftPressed = inputEvents.pressed(Keycode::LEFT_SHIFT);
            bool breakSelection = getSelectionLength() != 0 && !shiftPressed;
            stepDefaultDown(shiftPressed, breakSelection);
        };
    } else {
        onDownPressed = callback;
    }
}

void TextBox::setTextSupplier(wstringsupplier supplier) {
    this->supplier = std::move(supplier);
}

void TextBox::setTextConsumer(wstringconsumer consumer) {
    this->consumer = std::move(consumer);
}

void TextBox::setTextSubConsumer(wstringconsumer consumer) {
    this->subconsumer = std::move(consumer);
}

void TextBox::setTextValidator(wstringchecker validator) {
    this->validator = std::move(validator);
}

void TextBox::setOnControlCombination(key_handler handler) {
    this->controlCombinationsHandler = std::move(handler);
}

void TextBox::setFocusedColor(glm::vec4 color) {
    this->focusedColor = color;
}

glm::vec4 TextBox::getFocusedColor() const {
    return focusedColor;
}

void TextBox::setTextColor(glm::vec4 color) {
    this->textColor = color;
}

glm::vec4 TextBox::getTextColor() const {
    return textColor;
}

void TextBox::setErrorColor(glm::vec4 color) {
    this->invalidColor = color;
}

glm::vec4 TextBox::getErrorColor() const {
    return invalidColor;
}

const std::wstring& TextBox::getText() const {
    if (input.empty()) return placeholder;
    return input;
}

void TextBox::setText(const std::wstring& value) {
    this->input = value;
    input.erase(std::remove(input.begin(), input.end(), '\r'), input.end());
    historian->reset();
    history->clear();
    editedHistorySize = 0;
    refreshSyntax();
}

const std::wstring& TextBox::getPlaceholder() const {
    return placeholder;
}

void TextBox::setPlaceholder(const std::wstring& placeholder) {
    this->placeholder = placeholder;
}

const std::wstring& TextBox::getHint() const {
    return hint;
}

void TextBox::setHint(const std::wstring& text) {
    this->hint = text;
}

std::wstring TextBox::getSelection() const {
    const auto& text = label->getText();
    return text.substr(selectionStart, selectionEnd - selectionStart);
}

size_t TextBox::getCaret() const {
    return caret;
}

void TextBox::setCaret(size_t position) {
    const auto& labelText = label->getText();
    caret = std::min(static_cast<size_t>(position), input.length());
    if (font == nullptr) {
        return;
    }
    int width = label->getSize().x;

    rawTextCache.prepare(font, width);
    rawTextCache.update(input, multiline, label->isTextWrapping());

    caretLastMove = gui.getWindow().time();

    uint line = rawTextCache.getLineByTextIndex(caret);
    int offset = label->getLineYOffset(line) + getContentOffset().y;
    uint lineHeight = font->getLineHeight() * label->getLineInterval();
    if (scrollStep == 0) {
        scrollStep = lineHeight;
    }
    if (offset < 0) {
        scrolled(-glm::floor(offset / static_cast<double>(scrollStep) + 0.5f));
    } else if (offset >= getSize().y) {
        offset -= getSize().y;
        scrolled(-glm::ceil(offset / static_cast<double>(scrollStep) + 0.5f));
    }
    int lcaret = caret - rawTextCache.getTextLineOffset(line);
    int realoffset =
        font->calcWidth(labelText, lcaret) - static_cast<int>(textOffset) + 2;

    if (realoffset - width > 0) {
        setTextOffset(textOffset + realoffset - width);
    } else if (realoffset < 0) {
        setTextOffset(std::max(textOffset + realoffset, static_cast<size_t>(0))
        );
    }
}

void TextBox::setCaret(ptrdiff_t position) {
    if (position < 0) {
        setCaret(static_cast<size_t>(input.length() + position + 1));
    } else {
        setCaret(static_cast<size_t>(position));
    }
}

void TextBox::setPadding(glm::vec4 padding) {
    this->padding = padding;
    refresh();
}

glm::vec4 TextBox::getPadding() const {
    return padding;
}

void TextBox::setShowLineNumbers(bool flag) {
    showLineNumbers = flag;
}

bool TextBox::isShowLineNumbers() const {
    return showLineNumbers;
}

void TextBox::setSyntax(std::string_view lang) {
    syntax = lang;
    if (syntax.empty()) {
        label->setStyles(nullptr);
    } else {
        refreshSyntax();
    }
}

const std::string& TextBox::getSyntax() const {
    return syntax;
}

void TextBox::setMarkup(std::string_view lang) {
    markup = lang;
}

const std::string& TextBox::getMarkup() const {
    return markup;
}
