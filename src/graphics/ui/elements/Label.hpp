#pragma once

#include "UINode.hpp"
#include "constants.hpp"

#include "graphics/core/FontMetics.hpp"

class Font;
struct FontStylesScheme;

namespace gui {
    struct LineScheme {
        size_t offset;
        bool fake;
    };

    struct LabelCache {
        FontMetrics metrics;

        std::vector<LineScheme> lines;
        /// @brief Reset cache flag
        bool resetFlag = true;
        size_t wrapWidth = -1;
        int multilineWidth = 0;
    
        void prepare(const std::shared_ptr<Font>& font, FontMetrics metrics, size_t wrapWidth);
        void update(std::wstring_view text, bool multiline, bool wrap);

        size_t getTextLineOffset(size_t line) const;
        uint getLineByTextIndex(size_t index) const;
    };

    class Label final : public UINode {
        LabelCache cache;

        glm::vec2 calcSize();
    protected:
        std::wstring text;
        std::string fontName;
        wstringsupplier supplier = nullptr;
        
        /// @brief Lines interval multiplier
        float lineInterval = 1.5f;

        /// @brief Vertical alignment (only when multiline is set to false)
        Align valign = Align::CENTER;

        /// @brief Line separators and wrapping will be ignored if set to false
        bool multiline = false;

        /// @brief Text wrapping (works only if multiline is enabled)
        bool textWrap = true;
        
        /// @brief Text Y offset relative to label position
        /// (last calculated alignment)
        int textYOffset = 0;

        /// @brief Text line height multiplied by line interval
        int totalLineHeight = 1;

        /// @brief Auto resize label to fit text
        bool autoresize = false;

        /// @brief Text markup language
        std::string markup;

        std::unique_ptr<FontStylesScheme> styles;
    public:
        Label(GUI& gui, const std::string& text, std::string fontName="normal");
        Label(GUI& gui, const std::wstring& text, std::string fontName="normal");

        ~Label();

        void setText(std::wstring text);
        const std::wstring& getText() const;

        void setFontName(std::string name);
        const std::string& getFontName() const;

        /// @brief Set text vertical alignment (default value: center)
        /// @param align Align::top / Align::center / Align::bottom
        void setVerticalAlign(Align align);
        Align getVerticalAlign() const;

        /// @brief Get line height multiplier used for multiline labels 
        /// (default value: 1.5)
        float getLineInterval() const;

        /// @brief Set line height multiplier used for multiline labels
        void setLineInterval(float interval);

        /// @brief Get Y position of the text relative to label position
        /// @return Y offset
        int getTextYOffset() const;

        /// @brief Get Y position of the line relative to label position
        /// @param line target line index
        /// @return Y offset
        int getLineYOffset(uint line) const;

        /// @brief Get position of line start in the text
        /// @param line target line index
        /// @return position in the text [0..length]
        size_t getTextLineOffset(size_t line) const;

        /// @brief Get line index by its Y offset relative to label position
        /// @param offset target Y offset
        /// @return line index [0..+]
        uint getLineByYOffset(int offset) const;
        uint getLineByTextIndex(size_t index) const;
        uint getLinesNumber() const;
        bool isFakeLine(size_t line) const;

        void draw(const DrawContext& pctx, const Assets& assets) override;

        void textSupplier(wstringsupplier supplier);

        void setAutoResize(bool flag);
        bool isAutoResize() const;

        void setMultiline(bool multiline);
        bool isMultiline() const;

        void setTextWrapping(bool flag);
        bool isTextWrapping() const;

        void setMarkup(std::string_view lang);
        const std::string& getMarkup() const;

        void setStyles(std::unique_ptr<FontStylesScheme> styles);
    };
}
