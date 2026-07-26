# 3D Text

2D text displayed in 3D space.

The appearance of 3D text, is configured via a table, like [particles](particles.md). All fields are optional.

| Field           | Description                  | Default          |
| --------------- | ---------------------------- | ---------------- |
| display         | Display format               | static_billboard |
| color           | Text color                   | {1, 1, 1, 1}     |
| scale           | Text scale                   | 1                |
| render_distance | Text rendering distance      | 32               |
| xray_opacity    | Visibility through obstacles | 0                |
| perspective     | Perspective coefficient      | 1                |

Available display formats:

| Format            | Description                                                       |
| ----------------- | ----------------------------------------------------------------- |
| static_billboard  | Simple 3D text in the world with manual size and rotation control |
| y_free_billboard  | Freely rotating text on the Y axis facing the camera              |
| xy_free_billboard | Freely rotating text facing the camera                            |
| projected         | Projected text (displayed in screen coordinates)                  |

## *gfx.text3d* library

```lua
gfx.text3d.show(
    -- text position
    position: vec3,
    -- text to display
    text: str,
    -- text display settings table
    preset: table,
    -- additional text display settings table
    [optional] extension: table
) -> int
```

Creates 3D text, returning its id.

```lua
gfx.text3d.hide(id: int)
```

Removes 3D text.

```lua
gfx.text3d.get_text(id: int) -> str
gfx.text3d.set_text(id: int, text: str)
```

Text getter and setter.

```lua
gfx.text3d.get_pos(id: int) -> vec3
gfx.text3d.set_pos(id: int, pos: vec3)
```

Text position getter and setter.

```lua
gfx.text3d.get_axis_x(id: int) -> vec3
gfx.text3d.set_axis_x(id: int, pos: vec3)
```

Getter and setter of vector X.

```lua
gfx.text3d.get_axis_y(id: int) -> vec3
gfx.text3d.set_axis_y(id: int, pos: vec3)
```

Getter and setter of vector Y.

```lua
gfx.text3d.set_rotation(id: int, rotation: mat4)
```

Sets the text rotation (Sets the rotated vectors X,Y).

```lua
gfx.text3d.get_entity(id: int) -> int
gfx.text3d.set_entity(id: int, entity: int)
```

Getter and setter for the entity to which the 3D text position is bound.
The position set via `gfx.text3d.set_pos` is then relative to the entity's position.

```lua
gfx.text3d.update_settings(id: int, preset: table)
```

Updates text display settings.
