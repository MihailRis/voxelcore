#pragma once

#include "Container.hpp"

class UiDocument;

namespace gui {
    class InlineFrame final : public Container {
    public:
        explicit InlineFrame(GUI& gui);
        virtual ~InlineFrame();

        void setSrc(const std::string& src);
        void setDocument(const std::shared_ptr<UiDocument>& document);

        void act(float delta) override;
        void setSize(const glm::vec2& size) override;

        const std::string& getSrc() const;
    private:
        std::string src;
        std::shared_ptr<UiDocument> document;
        std::shared_ptr<UINode> root;
    };
}
