#pragma once

#include <memory>

class Assets;
class Camera;
class ModelBatch;

namespace rigging {
    struct Skeleton;
}

class HandsRenderer {
public:
    HandsRenderer(
        const Assets& assets,
        ModelBatch& modelBatch,
        std::shared_ptr<rigging::Skeleton> skeleton
    );

    void renderHands(const Camera& camera, float delta);
private:
    const Assets& assets;
    ModelBatch& modelBatch;
    std::shared_ptr<rigging::Skeleton> skeleton;
};
