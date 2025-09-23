#pragma once

#include "Panel.hpp"
#include "Label.hpp"

class Font;
class ActionsHistory;

namespace gui {
    class TextBoxHistorian;
    class TextBox : public Container {
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
        /// @brief Text input pointer, value may be greather than text length
        size_t caret = 0;
        /// @brief Actual local (line) position of the caret on vertical move
        size_t maxLocalCaret = 0;
        size_t textOffset = 0;
        int textInitX = 0;
        /// @brief Last time of the caret was moved (used for blink animation)
        double caretLastMove = 0.0;
        Font* font = nullptr;

        // Note: selection does not include markup
        size_t selectionStart = 0;
        size_t selectionEnd = 0;
        size_t selectionOrigin = 0;

        bool multiline = false;
        bool editable = true;
        bool autoresize = false;
        bool showLineNumbers = false;
        std::string markup;
        std::string syntax;

        void stepLeft(bool shiftPressed, bool breakSelection);
        void stepRight(bool shiftPressed, bool breakSelection);
        void stepDefaultDown(bool shiftPressed, bool breakSelection);
        void stepDefaultUp(bool shiftPressed, bool breakSelection);

        void onTab(bool shiftPressed);

        size_t normalizeIndex(int index);

        int calcIndexAt(int x, int y) const;
        void setTextOffset(uint x);
        bool eraseSelected();
        void resetSelection();
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

        virtual ~TextBox();
        
        void paste(const std::wstring& text, bool history=true);
        void erase(size_t start, size_t length);
            
        virtual void setTextSupplier(wstringsupplier supplier);

        /// @brief Consumer called on stop editing text (textbox defocus)
        /// @param consumer std::wstring consumer function
        virtual void setTextConsumer(wstringconsumer consumer);

        /// @brief Sub-consumer called while editing text
        /// @param consumer std::wstring consumer function
        virtual void setTextSubConsumer(wstringconsumer consumer);

        /// @brief Text validator called while text editing and returns true if
        /// text is valid
        /// @param validator std::wstring consumer returning boolean 
        virtual void setTextValidator(wstringchecker validator);

        virtual void setOnControlCombination(key_handler handler);

        virtual void setFocusedColor(glm::vec4 color);
        virtual glm::vec4 getFocusedColor() const;

        virtual void setTextColor(glm::vec4 color);
        virtual glm::vec4 getTextColor() const;

        /// @brief Set color of textbox marked by validator as invalid
        virtual void setErrorColor(glm::vec4 color);

        /// @brief Get color of textbox marked by validator as invalid
        virtual glm::vec4 getErrorColor() const;
        
        /// @brief Get TextBox content text or placeholder if empty
        virtual const std::wstring& getText() const;

        /// @brief Set TextBox content text
        virtual void setText(const std::wstring &value);

        /// @brief Get text placeholder
        virtual const std::wstring& getPlaceholder() const;

        /// @brief Set text placeholder
        /// @param text will be used instead of empty
        virtual void setPlaceholder(const std::wstring& text);

        /// @brief Get textbox hint
        virtual const std::wstring& getHint() const;

        /// @brief Set textbox hint
        /// @param text will be shown instead of empty
        virtual void setHint(const std::wstring& text);
        
        /// @brief Get selected text
        virtual std::wstring getSelection() const;

        /// @brief Get current caret position in text
        /// @return integer in range [0, text.length()]
        virtual size_t getCaret() const;

        /// @brief Set caret position in the text
        /// @param position integer in range [0, text.length()]
        virtual void setCaret(size_t position);

        /// @brief Set caret position in the text
        /// @param position integer in range [-text.length(), text.length()]
        virtual void setCaret(ptrdiff_t position);

        /// @brief Select part of the text
        /// @param start index of the first selected character
        /// @param end index of the last selected character + 1
        virtual void select(int start, int end);

        /// @brief Get number of line at specific position in text
        /// @param position target position
        /// @return line number
        virtual uint getLineAt(size_t position) const;

        /// @brief Get specific line text position
        /// @param line target line
        /// @return line position in text
        virtual size_t getLinePos(uint line) const;

        /// @brief Check text with validator set with setTextValidator
        /// @return true if text is valid
        virtual bool validate();

        virtual void setValid(bool valid);
        virtual bool isValid() const;

        /// @brief Enable/disable multiline mode        
        virtual void setMultiline(bool multiline);

        /// @brief Check if multiline mode is enabled 
        virtual bool isMultiline() const;

        /// @brief Enable/disable text wrapping        
        virtual void setTextWrapping(bool flag);

        /// @brief Check if text wrapping is enabled 
        virtual bool isTextWrapping() const;

        /// @brief Enable/disable text editing feature
        virtual void setEditable(bool editable);

        /// @brief Check if text editing feature is enabled 
        virtual bool isEditable() const;

        virtual bool isEdited() const;
        virtual void setUnedited();

        virtual void setPadding(glm::vec4 padding);
        glm::vec4 getPadding() const;

        size_t getSelectionStart() const;
        size_t getSelectionEnd() const;

        /// @brief Set runnable called on textbox focus
        virtual void setOnEditStart(runnable oneditstart);

        virtual void setAutoResize(bool flag);
        virtual bool isAutoResize() const;

        virtual void setShowLineNumbers(bool flag);
        virtual bool isShowLineNumbers() const;

        virtual void reposition() override;
        virtual void onFocus() override;
        virtual void refresh() override;
        virtual void doubleClick(int x, int y) override;
        virtual void click(int, int) override;
        virtual void mouseMove(int x, int y) override;
        virtual bool isFocuskeeper() const override {return true;}
        virtual void draw(const DrawContext& pctx, const Assets& assets) override;
        virtual void drawBackground(const DrawContext& pctx, const Assets& assets) override;
        virtual void typed(unsigned int codepoint) override; 
        virtual void keyPressed(Keycode key) override;
        virtual std::shared_ptr<UINode> getAt(const glm::vec2& pos) override;
        virtual void setOnUpPressed(const runnable &callback);
        virtual void setOnDownPressed(const runnable &callback);

        virtual void setSyntax(std::string_view lang);
        virtual const std::string& getSyntax() const;

        virtual void setMarkup(std::string_view lang);
        virtual const std::string& getMarkup() const;
    };
}
