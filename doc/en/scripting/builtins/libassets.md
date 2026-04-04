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
    -- Model file format (xml / vcm)
    format: str,
    -- Contents of the model file
    content: str,
    -- Model name after loading
    name: str,
    -- The skeleton name after loading. May be the same as the model name.
    -- The model will be split by the named bones.
    [optional]
    skeleton_name: string
)

-- Creates a Canvas from a loaded texture.
assets.to_canvas(
    -- The name of the loaded texture.
    -- Both standalone textures ("texture_name") and
    -- those in an atlas ("atlas:texture_name") are supported
    name: str
) --> Canvas
```
