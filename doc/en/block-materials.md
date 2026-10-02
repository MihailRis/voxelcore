# Block materials

A material is a common kit of properties applied to groups of blocks.
Material definitions are located in the `block_materials/` folder of a content pack.
A block refers to a material with the `material` property in the `pack:material_name` format
(see [block properties](block-properties.md)).

Example (`block_materials/glass.json`):

```json
{
    "steps-sound": "steps/glass",
    "place-sound": "blocks/glass_place",
    "break-sound": "blocks/glass_break",
    "sound-absorption": 0.3
}
```

## Properties

| Name             | Type   | Description                                              |
|------------------|--------|----------------------------------------------------------|
| steps-sound      | string | steps sound                                              |
| place-sound      | string | block placing sound                                      |
| break-sound      | string | block breaking sound                                     |
| hit-sound        | string | block hitting sound (`steps-sound` by default)           |
| sound-absorption | number | ambient sounds absorption when the camera is in the block|
| shader           | string | material shader name (see below)                         |

## Material shaders

A material shader allows to customize blocks and entities rendering without replacing the engine shaders.
Material shader code is included in the world shaders (opaque, translucent and shadows passes of blocks
and the entities program), so blocks with different materials are still rendered within a single draw call per chunk.

An entity uses the shader of its `material` property (see [entity properties](entity-properties.md)).
The material may be overridden at runtime with `rig:set_material` (see [skeleton](scripting/ecs.md)).

The `shader` property specifies the name of a shader located in the `shaders/materials/` folder of a pack.
Each of two files is optional, but an existing file must define its function:

- `shaders/materials/name.glslv` - vertex stage, must define the function:
    ```glsl
    // position - vertex world position
    vec3 material_vertex(vec3 position)
    ```
    The function may also modify the vertex normal `a_realnormal` (world space),
    for example when displacing a surface.
- `shaders/materials/name.glslf` - fragment stage, must define the function:
    ```glsl
    // color - block texture color before lighting applied
    vec4 material_fragment(vec4 color)
    ```

All the world shader uniforms (`u_timer`, `u_cameraPos`, `u_dayTime`, etc.),
vertex attributes (`v_position`, `v_texCoord`, `v_light`, `v_normal`) in the vertex stage and
varyings (`a_texCoord`, `a_modelpos`, `a_realnormal`, `a_skyLight`, etc.) in the fragment stage are available.
The same material shader code is compiled into all the programs (blocks, shadows and entities), so only
the common interface declared in `shaders/lib/world_vertex_header.glsl`, `world_fragment_header.glsl`
and `world_uniforms.glsl` may be used.
Shader parameters are declared with the `#param` directive, the same way as in [post-effects](scripting/builtins/libgfx-posteffects.md),
and controlled from scripts with the [gfx.materials](scripting/builtins/libgfx-materials.md) library.

Example (`shaders/materials/leaves.glslv`):

```glsl
#param float p_amplitude = 0.03
#param float p_speed = 1.2

vec3 material_vertex(vec3 position) {
    float phase = position.x * 0.7 + position.z * 1.1 + position.y * 0.4;
    position.x += sin(u_timer * p_speed + phase) * p_amplitude;
    return position;
}
```

Materials with the same `shader` share the shader code and its params.
Hook functions and params names are made unique for every shader automatically,
so different shaders may declare the same params.
Other functions and global variables of all the material shaders are compiled
into one program, so their names must be unique (use a prefix).

> [!NOTE]
> Compilation errors refer to a material shader by its index: `3:12` means line 12
> of the shader number 3. The indices are printed to the log on content loading,
> `128` is the generated dispatch code, `0` is the engine shader.

Limitations:

- up to 127 material shaders may be loaded at the same time
- array params are not supported, params names must not start with `_`
- the same param declared in both stages must have the same type
- vertex displacement must be continuous (depend on the vertex position only),
  otherwise gaps between faces appear; it must not depend on `u_cameraPos`,
  because the shadows pass is rendered from the light's point of view
- `texture()` calls inside `material_fragment` have undefined derivatives
  at borders between materials, use `textureLod` there
