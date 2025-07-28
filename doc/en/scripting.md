# Scripting

Project uses LuaJIT as a scripting language.

Subsections:
- [Engine events](scripting/events.md)
- [User input](scripting/user-input.md)
- [Filesystem and serialization](scripting/filesystem.md)
- [UI properties and methods](scripting/ui.md)
- [Entities and components](scripting/ecs.md)
- [Libraries](#)
    - [app](scripting/builtins/libapp.md)
    - [base64](scripting/builtins/libbase64.md)
    - [bjson, json, toml, yaml](scripting/filesystem.md)
    - [block](scripting/builtins/libblock.md)
    - [byteutil](scripting/builtins/libbyteutil.md)
    - [cameras](scripting/builtins/libcameras.md)
    - [entities](scripting/builtins/libentities.md)
    - [file](scripting/builtins/libfile.md)
    - [gfx.blockwraps](scripting/builtins/libgfx-blockwraps.md)
    - [gfx.particles](particles.md#gfxparticles-library)
    - [gfx.posteffects](scripting/builtins/libgfx-posteffects.md)
    - [gfx.text3d](3d-text.md#gfxtext3d-library)
    - [gfx.weather](scripting/builtins/libgfx-weather.md)
    - [gui](scripting/builtins/libgui.md)
    - [hud](scripting/builtins/libhud.md)
    - [input](scripting/builtins/libinput.md)
    - [inventory](scripting/builtins/libinventory.md)
    - [item](scripting/builtins/libitem.md)
    - [mat4](scripting/builtins/libmat4.md)
    - [network](scripting/builtins/libnetwork.md)
    - [pack](scripting/builtins/libpack.md)
    - [player](scripting/builtins/libplayer.md)
    - [quat](scripting/builtins/libquat.md)
    - [rules](scripting/builtins/librules.md)
    - [time](scripting/builtins/libtime.md)
    - [utf8](scripting/builtins/libutf8.md)
    - [vec2, vec3, vec4](scripting/builtins/libvecn.md)
    - [world](scripting/builtins/libworld.md)
- [Module core:bit_converter](scripting/modules/core_bit_converter.md)
- [Module core:data_buffer](scripting/modules/core_data_buffer.md)
- [Module core:vector2, core:vector3](scripting/modules/core_vector2_vector3.md)

## Type annotations

The documentation for Lua libraries uses type annotations,
not part of Lua syntax.

- vector - an array of three or four numbers
- vec2 - array of two numbers
- vec3 - array of three numbers
- vec4 - array of four numbers
- quat - array of four numbers - quaternion
- matrix - array of 16 numbers - matrix

## Core functions

```lua
require "packid:module_name" -- load Lua module from pack-folder/modules/
-- no extension included, just name
```
