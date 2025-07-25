#pragma once

#include <memory>
#include <string>
#include "Container.hpp"
#include "window/Camera.hpp"

class Batch3D;
class Framebuffer;

namespace gui {
    class ModelViewer : public Container {
    private:
        std::string modelName;
        Camera camera;
        std::unique_ptr<Batch3D> batch;
        std::unique_ptr<Framebuffer> fbo;

        glm::vec3 rotation {};
        glm::vec3 center {};
        float distance = 4.0f;
        bool grabbing = false;
    public:
        ModelViewer(GUI& gui, const glm::vec2& size, const std::string& modelName);

        ~ModelViewer();

        void setModel(const std::string& modelName);
        const std::string& getModel() const;

        Camera& getCamera();
        const Camera& getCamera() const;

        void act(float delta) override;
        void draw(const DrawContext& pctx, const Assets& assets) override;

        void setRotation(const glm::vec3& euler);
        void setCenter(const glm::vec3& center);
    };
}
