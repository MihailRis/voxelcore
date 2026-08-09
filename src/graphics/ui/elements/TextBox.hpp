#pragma once

#include "Panel.hpp"
#include "Label.hpp"

class Font;
class ActionsHistory;

namespace gui {
    class TextBoxHistorian;
    class TextBox final : public Container {
        const Input& inputEvents;
        LabelCache rawTextCache;
        std::shared_ptr<ActionsHistory> history;
        std::unique_ptr<TextBoxHistorian> historian;
        int editedHistorySize = 0;
    protected:
        glm::vec4 focusedColor {0.0f, 0.0f, 0.0f, 1.0f};
        glm::vec4 invalidColor {0.1f, 0.05f, 0.03f, 1.0f};
        glm::vec4 textColor {1.0f, 1.0f, 1.0f, 1.0f};
        glm::vec4 padding {2};
        std::shared_ptr<Label> label;
        std::shared_ptr<Label> lineNumbersLabel;
        /// @brief Current user input
        std::wstring input;
        /// @brief Text will be used if nothing entered
        std::wstring placeholder;
        /// @brief Text will be shown when nothing entered
        std::wstring hint;
        /// @brief Text supplier called every frame when not focused
        wstringsupplier supplier = nullptr;
        /// @brief Text supplier called on Enter pressed
        wstringconsumer consumer = nullptr;
        /// @brief Text supplier called while input
        wstringconsumer subconsumer = nullptr;
        /// @brief Text validator returning boolean value
        wstringchecker validator = nullptr;
        key_handler controlCombinationsHandler = nullptr;
        /// @brief Function called on focus
        runnable onEditStart = nullptr;
        /// @brief Function called on up arrow pressed
        runnable onUpPressed;
        /// @brief Function called on down arrow pressed
        runnable onDownPressed;
        /// @brief Is current input valid
        bool valid = true;
        /// @brief Text input pointer, value may be greater than text length
        size_t caret = 0;
        /// @brief Actual local (line) position of the caret on vertical move
        size_t maxLocalCaret = 0;
        size_t textOffset = 0;
        int textInitX = 0;
        /// @brief Last time of the caret was moved (used for blink animation)
        double caretLastMove = 0.0;

        // Note: selection does not include markup
        size_t selectionStart = 0;
        size_t selectionEnd = 0;
        size_t selectionOrigin = 0;

        bool multiline = false;
        bool editable = true;
        bool autoresize = false;
        bool showLineNumbers = false;
        bool keepLineSelection = false;
        std::string markup;
        std::string syntax;

        void stepCaret(bool shiftPressed, bool breakSelection, bool right);
        void stepDefaultDown(bool shiftPressed, bool breakSelection);
        void stepDefaultUp(bool shiftPressed, bool breakSelection);

        void onTab(bool shiftPressed);

        size_t normalizeIndex(int index);

        void setTextOffset(uint x);
        bool eraseSelected();
        void extendSelection(int index);
        void tokenSelectAt(int index);
        size_t getLineLength(uint line) const;

        /// @brief Get total length of the selection 
        size_t getSelectionLength() const;

        /// @brief Set maxLocalCaret to local (line) caret position
        void resetMaxLocalCaret();

        void performEditingKeyboardEvents(Keycode key);

        void refreshLabel();

        void onInput();

        void refreshSyntax();
    public:
        explicit TextBox(
            GUI& gui,
            std::wstring placeholder, 
            glm::vec4 padding=glm::vec4(4.0f)
        );

        ~TextBox();
        
        void paste(const std::wstring& text, bool history=true);
        void erase(size_t start, size_t length);
        void resetSelection();
            
        void setTextSupplier(wstringsupplier supplier);

        /// @brief Consumer called on stop editing text (textbox defocus)
        /// @param consumer std::wstring consumer function
        void setTextConsumer(wstringconsumer consumer);

        /// @brief Sub-consumer called while editing text
        /// @param consumer std::wstring consumer function
        void setTextSubConsumer(wstringconsumer consumer);

        /// @brief Text validator called while text editing and returns true if
        /// text is valid
        /// @param validator std::wstring consumer returning boolean 
        void setTextValidator(wstringchecker validator);

        void setOnControlCombination(key_handler handler);

        void setFocusedColor(glm::vec4 color);
        const glm::vec4& getFocusedColor() const;

        void setTextColor(glm::vec4 color);
        const glm::vec4& getTextColor() const;

        /// @brief Set color of textbox marked by validator as invalid
        void setErrorColor(glm::vec4 color);

        /// @brief Get color of textbox marked by validator as invalid
        glm::vec4 getErrorColor() const;
        
        /// @brief Get TextBox content text or placeholder if empty
        const std::wstring& getText() const;

        /// @brief Set TextBox content text
        void setText(const std::wstring &value);

        /// @brief Get text placeholder
        const std::wstring& getPlaceholder() const;

        /// @brief Set text placeholder
        /// @param text will be used instead of empty
        void setPlaceholder(const std::wstring& text);

        /// @brief Get textbox hint
        const std::wstring& getHint() const;

        /// @brief Set textbox hint
        /// @param text will be shown instead of empty
        void setHint(const std::wstring& text);
        
        /// @brief Get selected text
        std::wstring getSelection() const;

        /// @brief Get current caret position in text
        /// @return integer in range [0, text.length()]
        size_t getCaret() const;

        /// @brief Set caret position in the text
        /// @param position integer in range [0, text.length()]
        void setCaret(size_t position);

        /// @brief Set caret position in the text
        /// @param position integer in range [-text.length(), text.length()]
        void setCaret(ptrdiff_t position);

        /// @brief Select part of the text
        /// @param start index of the first selected character
        /// @param end index of the last selected character + 1
        void select(int start, int end);

        /// @brief Get number of line at specific position in text
        /// @param position target position
        /// @return line number
        uint getLineAt(size_t position) const;

        /// @brief Get specific line text position
        /// @param line target line
        /// @return line position in text
        size_t getLinePos(uint line) const;

        int calcIndexAt(int x, int y) const;
        int getLineYOffset(int line) const;

        /// @brief Check text with validator set with setTextValidator
        /// @return true if text is valid
        bool validate();

        void setValid(bool valid);
        bool isValid() const;

        /// @brief Enable/disable multiline mode        
        void setMultiline(bool multiline);

        /// @brief Check if multiline mode is enabled 
        bool isMultiline() const;

        /// @brief Enable/disable text wrapping        
        void setTextWrapping(bool flag);

        /// @brief Check if text wrapping is enabled 
        bool isTextWrapping() const;

        /// @brief Enable/disable text editing feature
        void setEditable(bool editable);

        /// @brief Check if text editing feature is enabled 
        bool isEditable() const;

        bool isEdited() const;
        void setUnedited();

        void setPadding(glm::vec4 padding);
        const glm::vec4& getPadding() const;

        size_t getSelectionStart() const;
        size_t getSelectionEnd() const;

        void setKeepLineSelection(bool flag);
        bool isKeepLineSelection() const;

        /// @brief Set runnable called on textbox focus
        void setOnEditStart(runnable oneditstart);

        void setAutoResize(bool flag);
        bool isAutoResize() const;

        void setShowLineNumbers(bool flag);
        bool isShowLineNumbers() const;

        void reposition() override;
        void onFocus() override;
        void refresh() override;
        void doubleClick(int x, int y) override;
        void click(int, int) override;
        void mouseMove(int x, int y) override;
        bool isFocuskeeper() const override {return true;}
        void draw(const DrawContext& pctx, const Assets& assets) override;
        void drawBackground(const DrawContext& pctx, const Assets& assets) override;
        void typed(unsigned int codepoint) override; 
        void keyPressed(Keycode key) override;
        std::shared_ptr<UINode> getAt(const glm::vec2& pos) override;
        void setOnUpPressed(const runnable& callback);
        void setOnDownPressed(const runnable& callback);

        void setSyntax(std::string_view lang);
        const std::string& getSyntax() const;

        void setMarkup(std::string_view lang);
        const std::string& getMarkup() const;

        std::shared_ptr<Label> getLabel() const;
    };
}
