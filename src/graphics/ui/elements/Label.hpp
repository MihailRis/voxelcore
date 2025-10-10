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
        ptrdiff_t fontId = 0;
        FontMetrics metrics;

        std::vector<LineScheme> lines;
        /// @brief Reset cache flag
        bool resetFlag = true;
        size_t wrapWidth = -1;
        int multilineWidth = 0;
    
        void prepare(std::ptrdiff_t fontId, FontMetrics metrics, size_t wrapWidth);
        void update(std::wstring_view text, bool multiline, bool wrap);

        size_t getTextLineOffset(size_t line) const;
        uint getLineByTextIndex(size_t index) const;
    };

    class Label : public UINode {
        LabelCache cache;

        glm::vec2 calcSize();
    protected:
        std::wstring text;
        std::string fontName;
        wstringsupplier supplier = nullptr;
        
        /// @brief Lines interval multiplier
        float lineInterval = 1.5f;

        /// @brief Vertical alignment (only when multiline is set to false)
        Align valign = Align::center;

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

        virtual ~Label();

        virtual void setText(std::wstring text);
        const std::wstring& getText() const;

        virtual void setFontName(std::string name);
        virtual const std::string& getFontName() const;

        /// @brief Set text vertical alignment (default value: center)
        /// @param align Align::top / Align::center / Align::bottom
        virtual void setVerticalAlign(Align align);
        virtual Align getVerticalAlign() const;

        /// @brief Get line height multiplier used for multiline labels 
        /// (default value: 1.5)
        virtual float getLineInterval() const;

        /// @brief Set line height multiplier used for multiline labels
        virtual void setLineInterval(float interval);

        /// @brief Get Y position of the text relative to label position
        /// @return Y offset
        virtual int getTextYOffset() const;

        /// @brief Get Y position of the line relative to label position
        /// @param line target line index
        /// @return Y offset
        virtual int getLineYOffset(uint line) const;

        /// @brief Get position of line start in the text
        /// @param line target line index
        /// @return position in the text [0..length]
        virtual size_t getTextLineOffset(size_t line) const;

        /// @brief Get line index by its Y offset relative to label position
        /// @param offset target Y offset
        /// @return line index [0..+]
        virtual uint getLineByYOffset(int offset) const;
        virtual uint getLineByTextIndex(size_t index) const;
        virtual uint getLinesNumber() const;
        virtual bool isFakeLine(size_t line) const;

        virtual void draw(const DrawContext& pctx, const Assets& assets) override;

        virtual void textSupplier(wstringsupplier supplier);

        virtual void setAutoResize(bool flag);
        virtual bool isAutoResize() const;

        virtual void setMultiline(bool multiline);
        virtual bool isMultiline() const;

        virtual void setTextWrapping(bool flag);
        virtual bool isTextWrapping() const;

        virtual void setMarkup(std::string_view lang);
        virtual const std::string& getMarkup() const;

        virtual void setStyles(std::unique_ptr<FontStylesScheme> styles);
    };
}
