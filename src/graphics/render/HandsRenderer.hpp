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

    void render(const Camera& camera);
private:
    const Assets& assets;
    ModelBatch& modelBatch;
    std::shared_ptr<rigging::Skeleton> skeleton;
};
