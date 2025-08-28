# *assets* library

A library for working with audio/visual assets.

## Functions

```lua
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
    name: str
)
```
