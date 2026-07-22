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
    - [assets](scripting/builtins/libassets.md)
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
    - [gfx.skeletons](scripting/builtins/libgfx-skeletons.md)
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
    - [pathfinding](scripting/builtins/libpathfinding.md)
    - [player](scripting/builtins/libplayer.md)
    - [quat](scripting/builtins/libquat.md)
    - [random](scripting/builtins/librandom.md)
    - [rules](scripting/builtins/librules.md)
    - [session](scripting/builtins/libsession.md)
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

## Namespaces

Currently, the engine uses a namespace hierarchy that minimizes conflicts between individual modules, scripts, and even packs.

The following structure is relevant:

- Global namespace (_G), which is the root namespace. Writing to it outside of engine core modules is highly undesirable, as it can waste hours of debugging time.
    - Pack namespace - created each time content is loaded for each pack included in the configuration. This namespace is used by modules, as well as item and block scripts.
        - [Component namespace](scripting/ecs.md#built-in-components).
        - [UI document namespace](scripting/ui.md).
- Isolated world generator namespace.

## Modules

A module is a global object with a lifetime limited to the lifetime of the loaded content, used both for interaction between different packages and as a substitute for global variables within the package itself.

The module must be located in `contentpack/modules/**` (subfolders are allowed, but you will need to specify them)

```lua
local module_name = require "contentpack:module_name" -- imports the module

-- If the module is in the same pack in which it is imported, you can use the shortcut:
local module_name = require "module_name"

-- When using nested folders:
local module_name = require "contentpack:path/to/module_name" -- the path does not include `modules`
local module_name = require "path/to/module_name" -- if in the same pack
```

The module will have the namespace of the folder in which it is located, regardless of the namespace from which the first import was performed.

When creating a module, you should follow the following approach, which is what *require* expects:

```lua
local this = {
    variable_name = ... -- public variables
}

-- private module variables
local variable_name = ...

-- private module functions
local function function_name(...)
    ...
end

-- public module functions
function this.function_name(...)
    ...
end

return this -- the result that require will return and cache
```

When reimported, the module will not be reloaded; instead, require will return the cached result.
