#include "HandsRenderer.hpp"

#include <glm/ext.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "ModelBatch.hpp"
#include "content/Content.hpp"
#include "graphics/commons/Model.hpp"
#include "objects/rigging.hpp"
#include "window/Camera.hpp"

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

void HandsRenderer::renderHands(
    const Camera& camera, float delta
) {
    auto& skeleton = *this->skeleton;
    const auto& config = *skeleton.config;

    // render
    modelBatch.setLightsOffset(camera.position);
    config.update(skeleton, glm::mat4(1.0f), glm::vec3());
    config.render(assets, modelBatch, skeleton, glm::mat4(1.0f), glm::vec3());
}
