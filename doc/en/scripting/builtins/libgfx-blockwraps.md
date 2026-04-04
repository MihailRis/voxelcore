# Library *gfx.blockwraps*

Library for working with *block wrappers*.

Block wrappers are introduced to implement block destruction animation and can be used for other purposes.

```lua
-- Creates a wrapper at the specified position, with the specified texture.
-- Returns the wrapper id.
gfx.blockwraps.wrap(
    -- block position
    position: vec3,
    -- wrap texture
    texture: string,
    -- emission [0.0; 1.0]
    [optional] emission: number = 1.0
) -> int

-- Removes the wrapper, if it exists.
gfx.blockwraps.unwrap(id: int)

-- Changes the position of the wrapper, if it exists.
gfx.blockwraps.set_pos(id: int, position: vec3)

-- Changes the texture of the wrapper, if it exists.
gfx.blockwraps.set_texture(id: int, texture: str)

-- Sets the face textures (-X, +X, -Y, +Y, -Z, +Z).
-- - nil - disables face rendering.
-- - "" - monotone texture.
gfx.blockwraps.set_faces(
    -- wrapper id
    id: int,
    -- face texture 1 (-X)
    face1?: string,
    -- face texture 2 (+X)
    face2?: string,
    ...
    -- face texture 6 (+Z)
    face6?: string
)

-- Sets the tint of the sides (-X, +X, -Y, +Y, -Z, +Z).
-- RGB [0.0; 1.0]
gfx.blockwraps.set_tints(
    -- wrapper id
    id: int,
    -- side 1 color (-X)
    face1?: vec3,
    -- side 2 color (+X)
    face2?: vec3,
    ...
    -- side 6 color (+Z)
    face6?: vec3
)
```

Wrappers will not be automatically removed without calling `unwrap`.
