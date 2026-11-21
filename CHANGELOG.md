# 0.33 - 2026.11.xx

[Documentation](https://github.com/MihailRis/VoxelEngine-Cpp/tree/release-0.33/doc/en/main-page.md) for 0.33

Table of contents:

- [Performance](#performance)
- [Added](#added)
- [New Functions](#new-functions)
- [Changes](#changes)
- [Fixes](#fixes)

## Performance

- optimized physics
- optimized pathfinding
- optimized entities render
- optimized skeleton animation
- optimized SSAO

## Added

- added blocks to base pack:
    - `abstract_stairs`
    - `brick_stairs`
- automatic missing content removal from UI
- sub-process interaction
- better entities semi-transparency
- VCA directives:
    - `@model` - bone model replacement
    - `@show` - visibility of a bone or entire skeleton/object
    - `@color` - color of the bone or entire skeleton/object (RGBA)
- world logic parameters:
    - random_ticks
    - block_ticks
- variant-based hitboxes
- `something/*` patterns in preload.json
- events:
    - can_be_placed block event
    - on_assets_loading content event
    - tps argument to on_world_tick event
- pathfinding:
    - agent width, height
    - direct paths where availble
- ui methods:
    - element:clearInterval
- `defaults.toml` parameters:
    - `inventory-layout`
    - `hotbar-layout`
- entity.properties table
- new entity properties:
    - `translucent`
- new block properties:
    - `ui-lock`
- entity rotation interpolation
- engine patch number
- vertices count info in debug panel

## New Functions

- app.delete_world_content
- app.get_stdin_stream
- app.get_stdout_stream
- assets.parse_skeleton
- block.get_hitbox
- skeleton:get_bone_offset
- hud.is_hotbar_visible
- hud.set_hotbar_visible
- pathfinding.set_width
- pathfinding.set_height
- player.get_hotbar_size
- player.set_hotbar_size
- world.get_logic
- world.set_logic

## Changes

- app.get_version now returns 3 numbers instead of 2
- removed EnTT dependency
- breaking: on_block_broken timing
- assets.parse_model now supports OBJ and VEC3 formats

## Fixes

- [animation fixes](https://github.com/MihailRis/voxelcore/commit/ef4207aed092bee8a0f1a658fadcae84d1324d51)
- [fix chunks render limits](https://github.com/MihailRis/voxelcore/commit/ef85592e21320bbc121d439c919379ba92e97f5e)
- [fix 'selectable' with static entities](https://github.com/MihailRis/voxelcore/commit/3c7755f61bab3e3311a089d8b99024ad081d1c10)
- [fix vca 'configure' directive parsing](https://github.com/MihailRis/voxelcore/commit/76e5e5b6f847e597d53180802957517701de7b8a)
- [fix 'mouse:' input prefix is not available](https://github.com/MihailRis/voxelcore/commit/cf35023adfb504cabf3bd825dd2079743f24f46f)
- [fix locked cursor if world was closed with menu hidden](https://github.com/MihailRis/voxelcore/commit/5060a8635deca8f8a9867fcd4006988f48a5b432)
- [fix outdated doc/*/scripting/builtins/libgfx-skeletons.md](https://github.com/MihailRis/voxelcore/commit/e8a83423d8d0c6bbc14f6974b7c05c106801ddbf)
- [fix reload_module ignoring pack environment](https://github.com/MihailRis/voxelcore/commit/d8017ff99553039e6d0114a033eb9c352076eaee)
- [fix zip archives paths and add zip files test](https://github.com/MihailRis/voxelcore/commit/fa5466f675a381861e20dcbb8c2e7ba33288f557)
- [fix crash in create_zip for files without modification time](https://github.com/MihailRis/voxelcore/commit/a56158b78e3ffcb7be7af2951375adb5dc4ac2e0)
- [fix shader info log is truncated](https://github.com/MihailRis/voxelcore/commit/bf98174e1531b70ff0d4be523e6f71195d1c595e)
- [fix '--test' run.sh](https://github.com/MihailRis/voxelcore/commit/bd3ad9f9644ceeb40c09c330cdf590ed5fff4978)
- [fix: entities frustum-culling not uses hitbox size (it uses entity scale only)](https://github.com/MihailRis/voxelcore/commit/40db547817c94b1476551053002e2e3455fda57a)
- [fix modelviewer background is not transparent](https://github.com/MihailRis/voxelcore/commit/e57e6a5cbee1543e43917ff5debde8d7732ce03d)
- [fix registry access in generator scripts](https://github.com/MihailRis/voxelcore/commit/ab129367adc2f515e9452d4928163df2715e0a04)
- [fix: missing network.request protocol filter](https://github.com/MihailRis/voxelcore/commit/6f466d52f263081c72595a61bc2efdf958e4acb9)
- [fix potential fatal errors](https://github.com/MihailRis/voxelcore/commit/f246619c6bbc2c144e36475cf7fcc16c3df21b73)
- [fix: model-primitives texture regions are broken](https://github.com/MihailRis/voxelcore/commit/7e41439e1ac4116e6d32b046923ab13e3cd20706)
- [fix player serialization](https://github.com/MihailRis/voxelcore/commit/73c16a2f586a94eaea13334b9149c6013d9b69ac)
- [fix docs typos and other trash](https://github.com/MihailRis/voxelcore/commit/ef61e28f50f96d049a0964e761966b5631180a8c)
- [fix animation & disable codegen trace](https://github.com/MihailRis/voxelcore/commit/0f71e1a15262b3daed887c46807e2fa572779282)
- [fix cyrillic in file path (Windows)](https://github.com/MihailRis/voxelcore/commit/f8101011a0383c8fe86bb126d440eeeca9c9afb5)
- [fix world.raycast](https://github.com/MihailRis/voxelcore/commit/28648381f1118c3e86d183ba01730f239af0e70c)
- [fix block library typos](https://github.com/MihailRis/voxelcore/commit/b49075347aadb7155c596802d3fa34ca9b61ca48)
- [fix block.set behaviour if world is not open](https://github.com/MihailRis/voxelcore/commit/bf55ffa183fcaec4be9441e377d1da7aaa6727eb)
- [fix project start script not actually started before first tick](https://github.com/MihailRis/voxelcore/commit/581702a836064a465d5e239bade6afb5becc28a4)
- [fix content-related app functions timings](https://github.com/MihailRis/voxelcore/commit/b2bbef699cfa4f6d5b2173e1c29d0efd1502fe9d)
- [fix entity.transform size animation](https://github.com/MihailRis/voxelcore/commit/59047f3c6aeb81905303b24b5647783dddc7bd5c)
- [fix app.config_packs adding default packs twice in a client app script](https://github.com/MihailRis/voxelcore/commit/3b0efb674a46ae7593dd5048409515ee76d912a2)
- [fix entities frustum culling](https://github.com/MihailRis/voxelcore/commit/c21871c3c94a99dc3164f5b6762ebbb55db9f6a4)
- [fix core:pathfinding with low refresh interval](https://github.com/MihailRis/voxelcore/commit/cb5bebf3ae69f8558a1e9aa68aecdc0cb94b5482)
- [fix content inheritance issues](https://github.com/MihailRis/voxelcore/commit/c9cc301e46266471c3f065872ba3f0933c2d80a4)
- [fix audio streams cleanup](https://github.com/MihailRis/voxelcore/commit/1bb66ebf1fc0114b92f5cd321a50255fdd5fb68f)
- [fix lighting settings based chunk meshes rebuilding](https://github.com/MihailRis/voxelcore/commit/c641a18d32b32c1412ff52fab6655354463b2547)
- [fix reverb room rolloff factor minimum value](https://github.com/MihailRis/voxelcore/commit/f4d4209e48fe5c3cc2989e0c8cd7d755c2912fe7)
- [fix fatal errors caused by incorrect interval event callbacks timing](https://github.com/MihailRis/voxelcore/commit/9be7cfa72425533641c05b3736bcdc8129db56bb)
- [fix headless tick rate: schedule ticks by deadlines](https://github.com/MihailRis/voxelcore/commit/67c913041e0d185b5e70dc1dc2f9cec4250af9a5)
- [fix math.random seed being the same on every launch](https://github.com/MihailRis/voxelcore/commit/457891594361f8a315a4c05e303010a4bb9a0c9c)
