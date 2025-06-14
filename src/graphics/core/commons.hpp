#pragma once

#include <string>
#include <optional>

#include "util/EnumMetadata.hpp"

enum class DrawPrimitive {
    point = 0,
    line,
    triangle,
};

enum class BlendMode {
    /// @brief Normal blending mode.
    normal, 
    /// @brief Additive blending mode.
    addition, 
    /// @brief Subtractive blending mode.
    inversion
};

/// @brief Standard GLFW 3.4 cursor shapes (same order and count as in GLFW).
/// It also works in GLFW 3.3 (unsupported shapes will be replaced with the arrow).
enum class CursorShape {
    /// @brief Regular arrow
    ARROW,
    /// @brief Text input I-beam
    TEXT,
    /// @brief Crosshair 
    CROSSHAIR,
    /// @brief Pointing hand
    POINTER,
    /// @brief Horizontal resize arrow
    EW_RESIZE,
    /// @brief Vertical resize arrow
    NS_RESIZE,
    
    // GLFW 3.4+ cursor shapes
    
    /// @brief Diagonal resize arrow (top-left to bottom-right)
    NWSE_RESIZE,
    /// @brief Diagonal resize arrow (top-right to bottom-left)
    NESW_RESIZE,
    /// @brief All-direction resize arrow
    ALL_RESIZE,
    /// @brief Operation not allowed
    NOT_ALLOWED,
    
    LAST=NOT_ALLOWED
};

VC_ENUM_METADATA(CursorShape)
    {"arrow", CursorShape::ARROW},
    {"text", CursorShape::TEXT},
    {"crosshair", CursorShape::CROSSHAIR},
    {"pointer", CursorShape::POINTER},
    {"ew-resize", CursorShape::EW_RESIZE},
    {"ns-resize", CursorShape::NS_RESIZE},
    {"nwse-resize", CursorShape::NWSE_RESIZE},
    {"nesw-resize", CursorShape::NESW_RESIZE},
    {"all-resize", CursorShape::ALL_RESIZE},
    {"not-allowed", CursorShape::NOT_ALLOWED},
VC_ENUM_END

class Flushable {
public:
    virtual ~Flushable() = default;

    virtual void flush() = 0;
};

class Bindable {
public:
    virtual ~Bindable() = default;

    virtual void bind() = 0;
    virtual void unbind() = 0;
};
