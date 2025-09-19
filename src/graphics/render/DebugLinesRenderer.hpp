#pragma once

class DrawContext;
class Camera;
class LineBatch;
class LinesRenderer;
class Shader;
class Level;

class DebugLinesRenderer {
public:
    static bool showPaths;
    
    DebugLinesRenderer(const Level& level)
        : level(level) {};

    /// @brief Render debug lines in the world
    /// @param ctx Draw context
    /// @param camera Camera used for rendering
    /// @param renderer Lines renderer used for rendering lines
    /// @param linesShader Shader used for rendering lines
    /// @param showChunkBorders Whether to show chunk borders
    void render(
        DrawContext& ctx,
        const Camera& camera,
        LinesRenderer& renderer,
        LineBatch& linesBatch,
        Shader& linesShader,
        bool showChunkBorders
    );
private:
    const Level& level;

    void drawBorders(
        LineBatch& batch, int sx, int sy, int sz, int ex, int ey, int ez
    );
    void drawCoordSystem(
        LineBatch& batch, const DrawContext& pctx, float length
    );

};
