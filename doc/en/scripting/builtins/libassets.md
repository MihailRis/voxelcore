# *assets* library

A library for working with audio/visual assets.

## Functions

```lua
-- Requests background texture loading
assets.request_texture(
    -- Image file
    filename: string,
    -- Texture name after loading
    name: string,
)
-- Loads a texture
assets.load_texture(
    -- Array of bytes of an image file
    data: table | Bytearray,
    -- Texture name after loading
    name: str,
    -- Image file format (only png is supported)
    [optional]
    format: str = "png"
)

-- Parses and loads a 3D model
assets.parse_model(
    -- Model file format (xml / vcm / obj / vec3)
    format: str,
    -- Contents of the model file
    content: str | Bytearray (.vec3),
    -- Model name after loading
    name: str,
    -- The skeleton name after loading. May be the same as the model name.
    -- The model will be split by the named bones.
    [optional]
    skeleton_name: string
)

-- Parses and loads a skeleton definition
assets.parse_skeleton(
    -- Skeleton file format (json / vcm / xml)
    format: str,
    -- Contents of the skeleton file
    content: str,
    -- Skeleton name after loading
    name: str
)

-- Parses and loads animation
assets.parse_animation(
    -- Animation file format (vca)
    format: string,
    -- Animation file content
    content: string,
    -- Animation name after loading
    name: string
)

-- Creates a Canvas from a loaded texture.
assets.to_canvas(
    -- The name of the loaded texture.
    -- Both standalone textures ("texture_name") and
    -- those in an atlas ("atlas:texture_name") are supported
    name: str
) --> Canvas
```
