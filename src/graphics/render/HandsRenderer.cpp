#include "HandsRenderer.hpp"

#include "animation/rigging.hpp"
#include "content/Content.hpp"
#include "graphics/commons/Model.hpp"
#include "ModelBatch.hpp"
#include "window/Camera.hpp"

#include <glm/ext.hpp>
#include <glm/gtc/matrix_transform.hpp>

using namespace rigging;

HandsRenderer::HandsRenderer(
    const Assets& assets,
    ModelBatch& modelBatch,
    std::shared_ptr<Skeleton> skeleton
)
    : assets(assets),
      modelBatch(modelBatch),
      skeleton(std::move(skeleton)) {
}

void HandsRenderer::render(const Camera& camera) {
    auto& skeleton = *this->skeleton;
    const auto& config = *skeleton.config;

    modelBatch.setLightsOffset(camera.position);
    config.render(
        assets,
        modelBatch,
        skeleton,
        glm::mat3(1.0f),
        glm::vec3(),
        glm::vec3(1.0f)
    );

    modelBatch.render();
    modelBatch.setLightsOffset(glm::vec3());
}
