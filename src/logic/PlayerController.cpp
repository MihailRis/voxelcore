#define _USE_MATH_DEFINES
#include "PlayerController.hpp"

#include <algorithm>
#include <cmath>

#include "BlocksController.hpp"
#include "content/Content.hpp"
#include "core_defs.hpp"
#include "engine/Engine.hpp"
#include "items/Inventory.hpp"
#include "items/ItemDef.hpp"
#include "items/ItemStack.hpp"
#include "lighting/Lighting.hpp"
#include "objects/Entities.hpp"
#include "objects/Player.hpp"
#include "objects/Players.hpp"
#include "physics/Hitbox.hpp"
#include "physics/PhysicsSolver.hpp"
#include "scripting/scripting.hpp"
#include "settings.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/voxel.hpp"
#include "window/Camera.hpp"
#include "window/Window.hpp"
#include "window/input.hpp"
#include "world/Level.hpp"

const float INTERACTION_RELOAD = 0.160f;
const float STEPS_SPEED = 2.2f;
const float CAM_SHAKE_OFFSET = 0.0075f;
const float CAM_SHAKE_OFFSET_Y = 0.031f;
const float CAM_SHAKE_SPEED = STEPS_SPEED;
const float CAM_SHAKE_DELTA_K = 10.0f;
const float ZOOM_SPEED = 16.0f;
const float CROUCH_ZOOM = 0.9f;
const float RUN_ZOOM = 1.1f;
const float C_ZOOM = 0.1f;
const float CROUCH_SHIFT_Y = -0.2f;

CameraControl::CameraControl(Player& player, const CameraSettings& settings)
    : player(player),
      camera(player.fpCamera),
      settings(settings),
      offset(0.0f, 0.7f, 0.0f) {
}

void CameraControl::refreshPosition() {
    camera->position = player.getPosition() + offset;
}

void CameraControl::refreshRotation() {
    const glm::vec3& rotation = player.getRotation();
    camera->rotation = glm::mat4(1.0f);
    camera->rotate(
        glm::radians(rotation.y),
        glm::radians(rotation.x),
        glm::radians(rotation.z)
    );
}

void CameraControl::updateMouse(PlayerInput& input, int windowHeight) {
    glm::vec3 rotation = player.getRotation();

    float sensitivity =
        (input.zoom ? settings.sensitivity.get() / 4.f
                    : settings.sensitivity.get());

    auto d = glm::degrees(
        input.delta / static_cast<float>(windowHeight) * sensitivity
    );
    rotation.x -= d.x;
    rotation.y -= d.y;

    if (rotation.y < -89.9f) {
        rotation.y = -89.9f;
    } else if (rotation.y > 89.9f) {
        rotation.y = 89.9f;
    }
    if (rotation.x > 180.f) {
        rotation.x -= 360.f;
    } else if (rotation.x < -180.f) {
        rotation.x += 360.f;
    }

    player.setRotation(rotation);

    camera->rotation = glm::mat4(1.0f);
    camera->rotate(
        glm::radians(rotation.y),
        glm::radians(rotation.x),
        glm::radians(rotation.z)
    );
}

glm::vec3 CameraControl::updateCameraShaking(
    const Hitbox& hitbox, float delta
) {
    glm::vec3 offset {};
    const float k = CAM_SHAKE_DELTA_K;
    const float ov = CAM_SHAKE_OFFSET_Y;
    const glm::vec3& vel = hitbox.velocity;

    if (settings.shaking.get()) {
        shake = shake * (1.0f - delta * k);
        float oh = CAM_SHAKE_OFFSET;
        if (hitbox.grounded) {
            float f = glm::length(glm::vec2(vel.x, vel.z));
            shakeTimer += delta * f * CAM_SHAKE_SPEED;
            shake += f * delta * k;
            oh *= glm::sqrt(f);
        }

        offset += camera->right * glm::sin(shakeTimer) * oh * shake;
        offset += camera->up * glm::abs(glm::cos(shakeTimer)) * ov * shake;
    }
    if (settings.inertia.get()) {
        interpVel = interpVel * (1.0f - delta * 5) + vel * delta * 0.1f;
        if (hitbox.grounded && interpVel.y < 0.0f) {
            interpVel.y *= -30.0f;
        }
        offset -= glm::min(interpVel * 0.05f, 1.0f);
    }
    return offset;
}

void CameraControl::updateFov(
    const Hitbox& hitbox, PlayerInput input, float delta, bool effects
) {
    bool crouch = input.shift && hitbox.grounded && !input.sprint;

    float dt = fmin(1.0f, delta * ZOOM_SPEED);
    float zoomValue = 1.0f;
    if (effects) {
        if (crouch) {
            offset += glm::vec3(0.f, CROUCH_SHIFT_Y, 0.f);
            zoomValue = CROUCH_ZOOM;
        } else if (input.sprint && (input.moveForward || input.moveBack || 
                                    input.moveLeft || input.moveRight)) {
            zoomValue = RUN_ZOOM;
        }
    }
    if (input.zoom) zoomValue *= C_ZOOM;
    camera->zoom = zoomValue * dt + camera->zoom * (1.0f - dt);
}

// temporary solution
// more extensible but uglier
void CameraControl::switchCamera() {
    const std::vector<std::shared_ptr<Camera>> playerCameras {
        camera, player.tpCamera, player.spCamera};

    auto index = std::distance(
        playerCameras.begin(),
        std::find_if(
            playerCameras.begin(),
            playerCameras.end(),
            [this](auto& ptr) {
                return ptr.get() == player.currentCamera.get();
            }
        )
    );
    if (static_cast<size_t>(index) != playerCameras.size()) {
        index = (index + 1) % playerCameras.size();
        player.currentCamera = playerCameras.at(index);
    } else {
        player.currentCamera = camera;
    }
}

void CameraControl::update(
    PlayerInput input, float delta, const Chunks& chunks
) {
    offset = glm::vec3(0.0f, 0.0f, 0.0f);

    if (auto hitbox = player.getHitbox()) {
        offset.y += hitbox->halfsize.y * (0.7f / 0.9f);
        if (!input.cheat) {
            offset += updateCameraShaking(*hitbox, delta);
        }
        updateFov(*hitbox, input, delta, settings.fovEffects.get());
    }
    if (input.cameraMode) {
        switchCamera();
    }

    const auto& spCamera = player.spCamera;
    const auto& tpCamera = player.tpCamera;

    refreshPosition();

    camera->updateVectors();
    if (player.currentCamera == spCamera) {
        spCamera->position =
            chunks.rayCastToObstacle(camera->position, camera->front, 3.0f) -
            0.4f * camera->front;
        spCamera->dir = -camera->dir;
        spCamera->front = -camera->front;
        spCamera->right = -camera->right;
    } else if (player.currentCamera == tpCamera) {
        tpCamera->position =
            chunks.rayCastToObstacle(camera->position, -camera->front, 3.0f) +
            0.4f * camera->front;
        tpCamera->dir = camera->dir;
        tpCamera->front = camera->front;
        tpCamera->right = camera->right;
    }
    if (player.currentCamera == spCamera || player.currentCamera == tpCamera ||
        player.currentCamera == camera) {
        player.currentCamera->setFov(glm::radians(settings.fov.get()));
    }
}

PlayerController::PlayerController(
    const EngineSettings& settings,
    Level& level,
    Player& player,
    BlocksController& blocksController
)
    : level(level),
      player(player),
      camControl(player, settings.camera),
      blocksController(blocksController) {
}

void PlayerController::onFootstep(const Hitbox& hitbox) {
    auto pos = hitbox.position;
    auto half = hitbox.halfsize;

    for (int offsetZ = -1; offsetZ <= 1; offsetZ++) {
        for (int offsetX = -1; offsetX <= 1; offsetX++) {
            int x = std::floor(pos.x + half.x * offsetX);
            int y = std::floor(pos.y - half.y * 1.1f);
            int z = std::floor(pos.z + half.z * offsetZ);
            auto vox = player.chunks->get(x, y, z);
            if (vox) {
                auto& def = level.content.getIndices()->blocks.require(vox->id);
                if (!def.obstacle) {
                    continue;
                }
                blocksController.onBlockInteraction(
                    &player, glm::ivec3(x, y, z), def, BlockInteraction::step
                );
                return;
            }
        }
    }
}

void PlayerController::updateFootsteps(float delta) {
    auto hitbox = player.getHitbox();
    if (hitbox && hitbox->grounded) {
        const glm::vec3& vel = hitbox->velocity;
        float f = glm::length(glm::vec2(vel.x, vel.z));
        stepsTimer += delta * f * STEPS_SPEED;
        if (stepsTimer >= M_PI) {
            stepsTimer = fmod(stepsTimer, M_PI);
            onFootstep(*hitbox);
        }
    } else {
        stepsTimer = M_PI;
    }
}

void PlayerController::update(float delta, const Input* inputEvents) {
    if (inputEvents) {
        updateKeyboard(*inputEvents);
        player.updateSelectedEntity();
    } else {
        resetKeyboard();
    }
    updatePlayer(delta);
}

void PlayerController::postUpdate(
    float delta, int windowHeight, const Input* input, bool pause
) {
    if (!pause) {
        updateFootsteps(delta);
    }

    if (!pause && input) {
        camControl.updateMouse(this->input, windowHeight);
    }
    camControl.refreshRotation();
    player.postUpdate();
    camControl.update(this->input, pause ? 0.0f : delta, *player.chunks);
    if (input) {
        updateInteraction(*input, delta);
    } else {
        player.selection = {};
    }
}

void PlayerController::updateKeyboard(const Input& inputEvents) {
    const auto& bindings = inputEvents.getBindings();
    input.moveForward = bindings.active(BIND_MOVE_FORWARD);
    input.moveBack = bindings.active(BIND_MOVE_BACK);
    input.moveLeft = bindings.active(BIND_MOVE_LEFT);
    input.moveRight = bindings.active(BIND_MOVE_RIGHT);
    input.sprint = bindings.active(BIND_MOVE_SPRINT);
    input.shift = bindings.active(BIND_MOVE_CROUCH);
    input.cheat = bindings.active(BIND_MOVE_CHEAT);
    input.jump = bindings.active(BIND_MOVE_JUMP);
    input.zoom = bindings.active(BIND_CAM_ZOOM);
    input.cameraMode = bindings.jactive(BIND_CAM_MODE);
    input.delta = inputEvents.getCursor().delta;
}

void PlayerController::resetKeyboard() {
    input.zoom = false;
    input.moveForward = false;
    input.moveBack = false;
    input.moveLeft = false;
    input.moveRight = false;
    input.sprint = false;
    input.shift = false;
    input.cheat = false;
    input.jump = false;
    input.delta = {};
}

void PlayerController::updatePlayer(float delta) {
    player.updateInput(input, delta);
}

static int determine_rotation(
    const Block* def, const glm::ivec3& norm, const glm::vec3& camDir
) {
    if (def && def->rotatable) {
        const std::string& name = def->rotations.name;
        if (name == "pipe") {
            if (norm.x < 0.0f) return BLOCK_DIR_WEST;
            if (norm.x > 0.0f) return BLOCK_DIR_EAST;
            if (norm.y > 0.0f) return BLOCK_DIR_UP;
            if (norm.y < 0.0f) return BLOCK_DIR_DOWN;
            if (norm.z > 0.0f) return BLOCK_DIR_NORTH;
            if (norm.z < 0.0f) return BLOCK_DIR_SOUTH;
        } else if (name == "pane" || name == "stairs") {
            int verticalBit = (name == "stairs" && (norm.y - camDir.y * 0.5f) < 0.0) ? 4 : 0; 
            if (abs(camDir.x) > abs(camDir.z)) {
                if (camDir.x > 0.0f) return BLOCK_DIR_EAST | verticalBit;
                if (camDir.x < 0.0f) return BLOCK_DIR_WEST | verticalBit;
            }
            if (abs(camDir.x) < abs(camDir.z)) {
                if (camDir.z > 0.0f) return BLOCK_DIR_SOUTH | verticalBit;
                if (camDir.z < 0.0f) return BLOCK_DIR_NORTH | verticalBit;
            }
        }
    }
    return 0;
}

voxel* PlayerController::updateSelection(float maxDistance) {
    auto indices = level.content.getIndices();
    auto& chunks = *player.chunks;
    auto camera = player.fpCamera.get();
    auto& selection = player.selection;

    glm::vec3 end;
    glm::ivec3 iend;
    glm::ivec3 norm;
    voxel* vox = chunks.rayCast(
        camera->position, camera->front, maxDistance, end, norm, iend
    );
    if (vox) {
        maxDistance = glm::distance(camera->position, end);
    }
    auto prevEntity = selection.entity;
    selection.entity = ENTITY_NONE;
    selection.actualPosition = iend;
    if (auto result = level.entities->rayCast(
            camera->position, camera->front, maxDistance, player.getEntity()
        )) {
        selection.entity = result->entity;
        selection.hitPosition =
            camera->position + camera->front * result->distance;
        selection.position = selection.hitPosition;
        selection.actualPosition = selection.position;
        selection.normal = result->normal;
    }
    if (selection.entity != prevEntity) {
        if (prevEntity != ENTITY_NONE) {
            if (auto pentity = level.entities->get(prevEntity)) {
                scripting::on_aim_off(*pentity, &player);
            }
        }
        if (selection.entity != ENTITY_NONE) {
            if (auto pentity = level.entities->get(selection.entity)) {
                scripting::on_aim_on(*pentity, &player);
            }
        }
    }
    if (vox == nullptr || selection.entity) {
        selection.vox = {BLOCK_VOID, {}};
        return nullptr;
    }
    blockstate selectedState = vox->state;
    selection.vox = *vox;
    if (selectedState.segment) {
        selection.position = chunks.seekOrigin(
            iend, indices->blocks.require(selection.vox.id), selectedState
        );
        auto origin = chunks.get(selection.position);
        if (origin && origin->id != vox->id) {
            chunks.set(iend.x, iend.y, iend.z, 0, {});
            return updateSelection(maxDistance);
        }
    } else {
        selection.position = iend;
    }
    selection.hitPosition = end;
    selection.normal = norm;
    return vox;
}

void PlayerController::processRightClick(
    const Block& def, const Block& target
) {
    const auto& selection = player.selection;
    auto& chunks = *player.chunks;
    auto camera = player.fpCamera.get();

    blockstate state {};
    state.rotation = determine_rotation(&def, selection.normal, camera->front);

    if (!input.shift && target.rt.funcsset.oninteract) {
        if (scripting::on_block_interact(
                &player, target, selection.actualPosition
            )) {
            return;
        }
    }
    glm::ivec3 coord = selection.actualPosition;
    if (!target.replaceable) {
        coord += selection.normal;
    } else if (def.rotations.name == BlockRotProfile::PIPE_NAME) {
        state.rotation = BLOCK_DIR_UP;
    }
    blockid_t chosenBlock = def.rt.id;

    if (def.obstacle) {
        const auto& hitboxes = def.rt.hitboxes[state.rotation];
        for (const AABB& blockAABB : hitboxes) {
            if (level.entities->hasBlockingInside(blockAABB.translated(coord)
                )) {
                return;
            }
        }
    }
    auto vox = chunks.get(coord);
    if (vox == nullptr) {
        return;
    }
    if (!chunks.checkReplaceability(def, state, coord)) {
        return;
    }
    if (def.grounded) {
        const auto& vec = get_ground_direction(def, state.rotation);
        if (!chunks.isSolidBlock(
                coord.x + vec.x, coord.y + vec.y, coord.z + vec.z
            )) {
            return;
        }
    }
    if (chosenBlock != vox->id && chosenBlock) {
        if (!player.isInfiniteItems()) {
            auto& slot = player.getInventory()->getSlot(player.getChosenSlot());
            slot.setCount(slot.getCount() - 1);
        }
        blocksController.placeBlock(
            &player, def, state, coord.x, coord.y, coord.z
        );
    }
}

void PlayerController::updateEntityInteraction(
    entityid_t eid, bool lclick, bool rclick
) {
    auto entityOpt = level.entities->get(eid);
    if (!entityOpt.has_value()) {
        return;
    }
    auto entity = *entityOpt;
    if (lclick) {
        scripting::on_attacked(entity, &player, player.getEntity());
    }
    if (rclick) {
        scripting::on_entity_used(entity, &player);
    }
}

void PlayerController::updateInteraction(const Input& inputEvents, float delta) {
    auto indices = level.content.getIndices();
    const auto& selection = player.selection;

    if (interactionTimer > 0.0f) {
        interactionTimer -= delta;
    }
    const auto& bindings = inputEvents.getBindings();
    bool xkey = bindings.active(BIND_PLAYER_FAST_INTERACTOIN);
    float maxDistance = xkey ? 200.0f : 10.0f;
    bool longInteraction = interactionTimer <= 0 || xkey;
    bool lclick = bindings.jactive(BIND_PLAYER_DESTROY) ||
                  (longInteraction && bindings.active(BIND_PLAYER_DESTROY));
    bool lattack = bindings.jactive(BIND_PLAYER_ATTACK);
    bool rclick = bindings.jactive(BIND_PLAYER_BUILD) ||
                  (longInteraction && bindings.active(BIND_PLAYER_BUILD));
    if (lclick || rclick) {
        interactionTimer = INTERACTION_RELOAD;
    }

    auto inventory = player.getInventory();
    const ItemStack& stack = inventory->getSlot(player.getChosenSlot());
    auto& item = indices->items.require(stack.getItemId());

    auto vox = updateSelection(maxDistance);
    if (vox == nullptr) {
        if (rclick && item.rt.funcsset.on_use) {
            scripting::on_item_use(&player, item);
        }
        if (selection.entity) {
            updateEntityInteraction(selection.entity, lattack, rclick);
        }
        return;
    }

    auto iend = selection.position;
    if (lclick && !input.shift && item.rt.funcsset.on_block_break_by) {
        if (scripting::on_item_break_block(
                &player, item, iend.x, iend.y, iend.z
            )) {
            return;
        }
    }
    auto& target = indices->blocks.require(vox->id);
    if (lclick) {
        scripting::on_block_breaking(&player, target, iend);
        if (player.isInstantDestruction() && target.breakable) {
            blocksController.breakBlock(
                &player, target, iend.x, iend.y, iend.z
            );
        }
    }
    if (rclick && !input.shift) {
        bool preventDefault = false;
        if (item.rt.funcsset.on_use_on_block) {
            preventDefault = scripting::on_item_use_on_block(
                &player, item, iend, selection.normal
            );
        } else if (item.rt.funcsset.on_use) {
            preventDefault = scripting::on_item_use(&player, item);
        }
        if (preventDefault) {
            return;
        }
    }
    auto def = indices->blocks.get(item.rt.placingBlock);
    if (def && rclick) {
        processRightClick(*def, target);
    }
}

Player* PlayerController::getPlayer() {
    return &player;
}
