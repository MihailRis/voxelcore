#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>

#include "objects/Player.hpp"
#include "util/Clock.hpp"

class Input;
class Engine;
class Camera;
class Level;
class Block;
class Chunks;
class BlocksController;
struct Hitbox;
struct CameraSettings;
struct EngineSettings;

class CameraControl {
    Player& player;
    std::shared_ptr<Camera> camera;
    const CameraSettings& settings;
    glm::vec3 offset;
    float shake = 0.0f;
    float shakeTimer = 0.0f;
    glm::vec3 interpVel {0.0f};

    /// @brief Update shaking timer and calculate camera offset
    /// @param delta delta time
    /// @return camera offset
    glm::vec3 updateCameraShaking(const Hitbox& hitbox, float delta);

    /// @brief Update field-of-view
    /// @param input player inputs
    /// @param delta delta time
    /// @param effects movement-related effects
    void updateFov(
        const Hitbox& hitbox, PlayerInput input, float delta, bool effects
    );

    /// @brief Switch active player camera
    void switchCamera();
public:
    CameraControl(Player& player, const CameraSettings& settings);
    void updateMouse(PlayerInput& input, int windowHeight);
    void update(PlayerInput input, float delta, const Chunks& chunks);
    void refreshPosition();
    void refreshRotation();
};

class PlayerController {
    Level& level;
    Player& player;
    PlayerInput input {};
    CameraControl camControl;
    BlocksController& blocksController;
    float interactionTimer = 0.0f;
    
    void updateKeyboard(const Input& inputEvents);
    void resetKeyboard();
    void updateEntityInteraction(entityid_t eid, bool lclick, bool rclick);
    void updateInteraction(const Input& inputEvents, float delta);

    float stepsTimer = 0.0f;
    void onFootstep(const Hitbox& hitbox);
    void updateFootsteps(float delta);
    void processRightClick(const Block& def, const Block& target);

    voxel* updateSelection(float maxDistance);
public:
    PlayerController(
        const EngineSettings& settings,
        Level& level,
        Player& player,
        BlocksController& blocksController
    );

    /// @brief Called after blocks update if not paused
    /// @param delta delta time
    /// @param inputEvents nullable window inputs
    void update(float delta, const Input* inputEvents);

    /// @brief Called after whole level update
    /// @param delta delta time
    /// @param inputEvents nullable window inputs
    /// @param pause is game paused
    void postUpdate(
        float delta, int windowHeight, const Input* inputEvents, bool pause
    );
    Player* getPlayer();
};
