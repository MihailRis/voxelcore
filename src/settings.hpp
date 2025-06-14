#pragma once

#include <string>
#include <unordered_map>

#include "data/setting.hpp"
#include "constants.hpp"
#include "typedefs.hpp"

struct AudioSettings {
    /// @brief try to initialize AL backend
    FlagSetting enabled {true};
    
    NumberSetting volumeMaster {1.0f, 0.0f, 1.0f, setting_format::percent};
    NumberSetting volumeRegular {1.0f, 0.0f, 1.0f, setting_format::percent};
    NumberSetting volumeUI {1.0f, 0.0f, 1.0f, setting_format::percent};
    NumberSetting volumeAmbient {1.0f, 0.0f, 1.0f, setting_format::percent};
    NumberSetting volumeMusic {1.0f, 0.0f, 1.0f, setting_format::percent};
};

struct DisplaySettings {
    /// @brief Is window in full screen mode
    FlagSetting fullscreen {false};
    /// @brief Window width (pixels)
    IntegerSetting width {1280};
    /// @brief Window height (pixels)
    IntegerSetting height {720};
    /// @brief Anti-aliasing samples
    IntegerSetting samples {0};
    /// @brief Framerate limit
    IntegerSetting framerate {-1, -1, 120};
    /// @brief Limit framerate when window is iconified
    FlagSetting limitFpsIconified {false};
};

struct ChunksSettings {
    /// @brief Max milliseconds that engine uses for chunks loading only
    IntegerSetting loadSpeed {4, 1, 32};
    /// @brief Radius of chunks loading zone (chunk is unit) 
    IntegerSetting loadDistance {22, 3, 80};
    /// @brief Buffer zone where chunks are not unloading (chunk is unit)
    IntegerSetting padding {2, 1, 8};
};

struct CameraSettings {
    /// @brief Camera dynamic field of view effects
    FlagSetting fovEffects {true};
    /// @brief Camera movement shake
    FlagSetting shaking {true};
    /// @brief Camera movement inertia effect
    FlagSetting inertia {true};
    /// @brief Camera field of view
    NumberSetting fov {90.0f, 10, 120};
    /// @brief Camera sensitivity
    NumberSetting sensitivity {2.0f, 0.1f, 10.0f};
};

struct GraphicsSettings {
    /// @brief Fog opacity is calculated as `pow(depth*k, fogCurve)` where k depends on chunksLoadDistance.
    /// 1.0 is linear, 2.0 is quadratic
    NumberSetting fogCurve {1.0f, 1.0f, 6.0f};
    /// @brief Lighting gamma
    NumberSetting gamma {1.0f, 0.4f, 1.0f};
    /// @brief Enable blocks backlight to prevent complete darkness
    FlagSetting backlight {true};
    /// @brief Disable culling with 'optional' mode
    FlagSetting denseRender {true};
    /// @brief Enable chunks frustum culling
    FlagSetting frustumCulling {true};
    /// @brief Skybox texture face resolution
    IntegerSetting skyboxResolution {64 + 32, 64, 128};
    /// @brief Chunk renderer vertices buffer capacity
    IntegerSetting chunkMaxVertices {200'000, 0, 4'000'000};
    /// @brief Chunk renderer vertices buffer capacity in dense render mode
    IntegerSetting chunkMaxVerticesDense {800'000, 0, 8'000'000};
    /// @brief Limit of chunk renderers count
    IntegerSetting chunkMaxRenderers {6, -4, 32};
    /// @brief Advanced render pipeline
    FlagSetting advancedRender {true};
    /// @brief Shadows quality
    IntegerSetting shadowsQuality {0, 0, 3};
};

struct DebugSettings {
    /// @brief Turns off chunks saving/loading
    FlagSetting generatorTestMode {false};
    /// @brief Write lights cache
    FlagSetting doWriteLights {true};
};

struct UiSettings {
    StringSetting language {"auto"};
    IntegerSetting worldPreviewSize {64, 1, 512};
};

struct NetworkSettings {
};

struct EngineSettings {
    AudioSettings audio;
    DisplaySettings display;
    ChunksSettings chunks;
    CameraSettings camera;
    GraphicsSettings graphics;
    DebugSettings debug;
    UiSettings ui;
    NetworkSettings network;
};
