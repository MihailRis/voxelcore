# 0.29 - 2025.09.20

[Documentation](https://github.com/MihailRis/VoxelEngine-Cpp/tree/release-0.29/doc/en/main-page.md) for 0.29

Table of contents:

- [Added](#added)
    - [Functions](#functions)
    - [Changes](#changes)
- [Fixes](#fixes)

## Added

- pathfinding
- components:
    - core:pathfinding
    - core:player
    - core:mob
- libraries:
    - random
    - gfx.skeletons
    - (documented) assets
- udp support
- schedules
- events:
    - on_physics_update (components)
    - on_block_tick(x, y, z, tps) (blocks)
- custom hand controller
- http headers
- named pipes
- optimizations:
    - speed up block.set
    - speed up vectors
- items description
- item properties methods
- tab + shift+tab
- blocks, items tags
- pack dependencies versions
- ~~allow to disable autospawn position~~ use player.set_spawnpoint
- entity.spawn command
- project script
- gui.root document
- time.schedules.world.common: Schedule

### Changes

- app.sleep_until - added 'timeout argument'
- network.get / post - added 'data' argument to error callback
- autorefresh model preview
- move player controls to lua
- move hand control to lua

### Functions

- block.model_name
- block.has_tag
- item.has_tag
- item.description
- base64.encode_urlsafe
- base64.decode_urlsafe
- vec2.rotate
- vecn.distance
- vecn.mix
- rigidbody:get_vdamping
- rigidbody:set_vdamping
- entity:require_component
- network.udp_connect
- random.random
- random.bytes
- random.uuid
- Random:random
- Random:seed
- hud.hand_controller
- inventory.get_caption
- inventory.set_caption
- inventory.get_description
- inventory.set_description
- pathfinding.create_agent
- pathfinding.remove_agent
- pathfinding.set_enabled
- pathfinding.is_enabled
- pathfinding.make_route
- pathfinding.make_route_async
- pathfinding.pull_route
- pathfinding.set_max_visited
- pathfinding.avoid_tag
- gfx.skeletons.get
- Skeleton:index
- Skeleton:get_model
- Skeleton:set_model
- Skeleton:get_matrix
- Skeleton:set_matrix
- Skeleton:get_texture
- Skeleton:set_texture
- Skeleton:is_visible
- Skeleton:set_visible
- Skeleton:get_color
- Skeleton:set_color
- Schedule:set_timeout(time_ms, callback)
- Schedule:set_interval(interval_ms, callback, [optional] repetions): int
- Schedule:remove_interval(id)
- ScheduleGroup:publish(schedule: Schedule)

## Fixes

- fix 3d text position / culling
- fix fragment:place rotation (#593)
- fix server socket creation in macos
- fix: base packs not scanned for app scripts
- fix lua::getfield and events registering
- fix UIDocument::rebuildIndices
- fix input library in headless mode
- fix rigidbody:set_gravity_scale
- fix extended blocks destruction particles spawn spread, offset
- fix shaders recompiling
- fix: C++ vecn functions precision loss
- fix coroutines errors handling
- fix: viewport size on toggle fullscreen
- fix: fullscreen monitor refresh rate
- fix: content menu panel height
- fix generation.create_fragment (#596)
- fix bytearray:insert (#594)
- fix: script overriding
- fix: hud.close after hud.show_overlay bug
- fix: 'cannot resume dead coroutine' (#569)
- fix: skybox is not visible behind translucent blocks
- fix: sampler arrays inbdexed with non-constant / uniform-based expressions are forbidden
- fix initial weather intensity
- fix drop count (560)
- fix BasicParser::parseNumber() out of range (560)
- fix rotation interpolation (#557)
