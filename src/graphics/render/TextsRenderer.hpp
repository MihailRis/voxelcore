#pragma once

#include <unordered_map>
#include <memory>

#include "typedefs.hpp"

class DrawContext;
class Camera;
class Assets;
class Batch3D;
class Frustum;
class TextNote;
struct EngineSettings;
class Level;

class TextsRenderer {
    const Level& level;
    Batch3D& batch;
    const Assets& assets;
    const Frustum& frustum;

    std::unordered_map<u64id_t, std::unique_ptr<TextNote>> notes;
    u64id_t nextNote = 1;

    void renderNote(
        const TextNote& note,
        const DrawContext& context,
        const Camera& camera,
        const EngineSettings& settings,
        bool hudVisible,
        bool frontLayer,
        bool projected
    );
public:
    TextsRenderer(
        const Level& level,
        Batch3D& batch,
        const Assets& assets,
        const Frustum& frustum
    );

    void render(
        const DrawContext& context,
        const Camera& camera,
        const EngineSettings& settings,
        bool hudVisible,
        bool frontLayer
    );

    u64id_t add(std::unique_ptr<TextNote> note);

    TextNote* get(u64id_t id) const;

    void remove(u64id_t id);
};
