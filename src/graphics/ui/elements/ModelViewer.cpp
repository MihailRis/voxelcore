#include <array>
#include <glm/ext.hpp>
#include "ModelViewer.hpp"

#include "assets/Assets.hpp"
#include "assets/assets_util.hpp"
#include "graphics/commons/Model.hpp"
#include "graphics/core/Batch2D.hpp"
#include "graphics/core/Batch3D.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/Framebuffer.hpp"
#include "graphics/core/DrawContext.hpp"
#include "window/Window.hpp"
#include "../GUI.hpp"

// TODO: remove
#include <GL/glew.h>

using namespace gui;

ModelViewer::ModelViewer(
    GUI& gui, const glm::vec2& size, const std::string& modelName
)
    : Container(gui, size),
      modelName(modelName),
      camera(),
      batch(std::make_unique<Batch3D>(1024)),
      fbo(std::make_unique<Framebuffer>(size.x, size.y)) {
    camera.perspective = true;
    camera.position = glm::vec3(2, 2, 2);
}

ModelViewer::~ModelViewer() = default;

void ModelViewer::setModel(const std::string& modelName) {
    this->modelName = modelName;
}

const std::string& ModelViewer::getModel() const {
    return modelName;
}

Camera& ModelViewer::getCamera() {
    return camera;
}

const Camera& ModelViewer::getCamera() const {
    return camera;
}

void ModelViewer::act(float delta) {
    Container::act(delta);

    auto& input = gui.getInput();
    
    if (!grabbing && hover && input.jclicked(Mousecode::BUTTON_3)) {
        grabbing = true;
    }
    if (grabbing && input.clicked(Mousecode::BUTTON_3)) {
        auto cursor = input.getCursor();
        if (input.pressed(Keycode::LEFT_SHIFT)) {
            center -= camera.right * (cursor.delta.x / size.x) * distance;
            center += camera.up * (cursor.delta.y / size.y) * distance;
        } else {
            rotation.x -= cursor.delta.x / size.x * glm::two_pi<float>();
            rotation.y -= cursor.delta.y / size.y * glm::two_pi<float>();
            rotation.y = glm::max(
                -glm::half_pi<float>(), glm::min(glm::half_pi<float>(), rotation.y)
            );
        }
    } else if (grabbing) {
        grabbing = false;
    }
    if (hover) {
        distance *= 1.0f - 0.2f * input.getScroll();
    }
    camera.rotation = glm::mat4(1.0f);
    camera.rotate(rotation.y, rotation.x, rotation.z);
    camera.position = center - camera.front * distance;
}

void ModelViewer::draw(const DrawContext& pctx, const Assets& assets) {
    camera.setAspectRatio(size.x / size.y);
    camera.updateVectors();
    
    auto model = assets.get<model::Model>(modelName);
    if (model == nullptr) {
        return;
    }
    auto& prevShader = Shader::getUsed();
    
    fbo->resize(size.x, size.y);
    {
        glDisable(GL_SCISSOR_TEST);
        auto ctx = pctx.sub();
        ctx.setFramebuffer(fbo.get());
        ctx.setViewport({size.x, size.y});
        ctx.setDepthTest(true);
        ctx.setCullFace(true);
        display::clear();
        
        auto& ui3dShader = assets.require<Shader>("ui3d");
        ui3dShader.use();
        ui3dShader.uniformMatrix("u_apply", glm::mat4(1.0f));
        ui3dShader.uniformMatrix("u_projview", camera.getProjView());
        batch->begin();
        for (const auto& mesh : model->meshes) {
            util::TextureRegion region;
            if (!mesh.texture.empty() && mesh.texture[0] == '$') {
                // todo: refactor
                static std::array<std::string, 6> faces {
                    "blocks:dbg_north",
                    "blocks:dbg_south",
                    "blocks:dbg_top",
                    "blocks:dbg_bottom",
                    "blocks:dbg_west",
                    "blocks:dbg_east",
                };
                region = util::get_texture_region(
                    assets,
                    faces.at(mesh.texture.at(1) - '0'),
                    "blocks:notfound"
                );
            } else {
                region = util::get_texture_region(assets, mesh.texture, "blocks:notfound");
            }
            batch->texture(region.texture);
            batch->setRegion(region.region);
            for (const auto& vertex : mesh.vertices) {
                batch->vertex(vertex.coord, vertex.uv, vertex.normal);
            }
        }
        batch->flush();
        glEnable(GL_SCISSOR_TEST);
    }

    auto pos = calcPos();
    prevShader.use();
    auto& batch2d = *pctx.getBatch2D();
    batch2d.texture(fbo->getTexture());
    batch2d.rect(pos.x, pos.y, size.x, size.y, 0.0f, 0.0f, 0.0f, UVRegion {}, false, true, glm::vec4{1.0f});

    Container::draw(pctx, assets);
}

void ModelViewer::setCenter(const glm::vec3& center) {
    this->center = center;
}

void ModelViewer::setRotation(const glm::vec3& euler) {
    this->rotation = euler;
}
