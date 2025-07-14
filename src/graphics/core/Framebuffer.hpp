#pragma once

#include "typedefs.hpp"
#include "commons.hpp"

#include <memory>

class Texture;

class Framebuffer : public Bindable {
    uint fbo;
    uint depth;
    uint width;
    uint height;
    uint format;
    std::unique_ptr<Texture> texture;
public:
    Framebuffer(uint fbo, uint depth, std::unique_ptr<Texture> texture);
    Framebuffer(uint width, uint height, bool alpha=false);
    ~Framebuffer();

    /// @brief Use framebuffer
    void bind() override;

    /// @brief Stop using framebuffer
    void unbind() override;

    /// @brief Update framebuffer texture size
    /// @param width new width
    /// @param height new height
    void resize(uint width, uint height);

    /// @brief Get framebuffer color attachment
    Texture* getTexture() const;

    /// @brief Get framebuffer width
    uint getWidth() const;
    /// @brief Get framebuffer height
    uint getHeight() const;

    uint getFBO() const;
};
