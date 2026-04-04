# 0.31 - 2026.03.22

[Documentation](https://github.com/MihailRis/VoxelEngine-Cpp/tree/release-0.31/doc/en/main-page.md) for 0.31

Table of contents:

- [New Features](#new-features)
- [Added](#added)
- [New Functions](#new-functions)
- [Changes](#changes)
- [Fixes](#fixes)


## New Features

- solid entities with solid hitboxes
- rendering ui to texture with 'frame' element
- loading vector fonts support with freetype library
- defining skeletons in VCM files
- project permissions system
- executing commands from stdin ('--stdin-cmd' command line argument enables it)

## Added

- added clouds, so sky is no more so deadly clear
- ui:
    - added 'onmouseenter', 'onmouseleave' events
    - added 'zIndex' common scripting property
    - added 'inventory' scripting property to slot element
    - added 'font' attribute
    - added 'text-align' label attribute
    - added 'frame' element
    - added 'fallback' image attribute
- particles:
    - added 'spawn_offset' property
- audio:
    - added acoustic effects (reverb)
    - added 'audio.acoustic-effects' setting
    - added 'record-audio' project permission
- projects system:
    - added project permissions configuration
    - added 'debugging' permission
- added 'base:flat_grass' world generator
- added ssao quality setting
- added background assets loader
- vcm format:
    - added 'bone' (model/skeleton node) element
    - added 'tri' (triangle) element
- added 'on_inventory_interact' hud event
- added weather properties:
    - 'sky_tint'
    - 'clouds_tint'
    - 'min_sky_light'
- added block properties:
    - 'grounding-behaviour' (extended blocks)
    - explicit 'solid' visual property
- added world generator parameters:
    - 'player-spawn-radius'
    - 'player-min-spawn-height'
    - 'player-max-spawn-height'
- added 'material' entity property
- added patch number to engine build produced by release pipeline
- new functions arguments:
    - added 'skeleton_name' argument to assets.parse_model
    - added 'include_non_selectable' argument to block.raycast
    - added 'variant_index' argument to block.get_model, block.model_name, block.get_textures
    - added optional 'emission' argument to gfx.blockwraps.wrap
    - added gui.load_document namespace extension table 
- added freetype dependency

## New Functions

- assets.request_texture
- block.get_hitbox
- entity.rigidbody:get_elasticity
- entity.rigidbody:get_mass
- entity.rigidbody:get_material
- entity.rigidbody:set_elasticity
- entity.rigidbody:set_mass
- entity.rigidbody:set_material
- entity.rigidbody:get_ground_vel
- gfx.blockwraps.set_faces
- gfx.blockwraps.set_tints
- gui.close_menu
- gui.create_frame
- gui.get_active_frame
- gui.set_active_frame
- hud.get_second_inventory
- hud.is_player_inventory_open
- mat4.perspective
- quat.from_euler
- session.get
- session.has
- session.reset

now documented:

- audio.input.get_input_info
- gui.ask
- gui.show_message

## Changes

- minor visual restoration
- assets.parse_model now supports OBJ format
- chunks loading optimizations
- app.load_content, app.reset_content, app.reconfig_packs and app.config_packs calling limited to app script coroutine and time.post_runnable context
- debug panel updated a bit
- pass app library from standard pages loader
- skeletons are now assets

## Fixes

- [fix vec3 shadeless flag](https://github.com/MihailRis/voxelcore/commit/5e05b44c26c1b57aaceb5e3478ccf5f98e3f14d7)
- [fix: missing extended block size limit check](https://github.com/MihailRis/voxelcore/commit/9628177698b29488965bffc60fa305e752245c13)
- [fix extended block grounding](https://github.com/MihailRis/voxelcore/commit/0b7dc14f5512478c7b7df408bab007d06de6f9ae)
- [fix blocks variants](https://github.com/MihailRis/voxelcore/commit/0cc8c70a1704bf511635c1c1e19fe707e8477127)
- [fix fatal exception caused by empty skeleton model texture name](https://github.com/MihailRis/voxelcore/commit/fc04284b34d5dfc8afd008933b61644dda6b73c6)
- [fix blocks random tick](https://github.com/MihailRis/voxelcore/commit/9ac8406558ebe6183071545aeb8e34a968503c18)
- [fix 'attempt to compare string with number' in audio.input.fetch](https://github.com/MihailRis/voxelcore/commit/ba2546e096c93e13fc0abfcf0e0447eb1721960f)
- [fix: FFI library access from non-built-in scripts vulnerability](https://github.com/MihailRis/voxelcore/commit/a32845d083f3d2528f04eebe173ed7358cd2cf14)
- [fix non-lighted chunks saving](https://github.com/MihailRis/voxelcore/commit/5b8c2d6162092f3655e2d857fa75ee42d879e604)
- [fix: attempting to build lights in non-local players chunk matrices](https://github.com/MihailRis/voxelcore/commit/3308a61d560c2d9bbff0c81d2ed5d391103ec5d0)
- [fix extended blocks collision detection](https://github.com/MihailRis/voxelcore/commit/c8e884009bdc7b7ee8025b05cfb12284f340d0af)
- [fix: block inventory not removing in some cases](https://github.com/MihailRis/voxelcore/commit/e4d21c940be9048076180976eaba31f0138661a3)
- [fix extended blocks culling on chunk borders](https://github.com/MihailRis/voxelcore/commit/7abbe43c3cf21bbf7afa3cf392b08e52950fa163)
- [fix: player name text is still visible when suspended](https://github.com/MihailRis/voxelcore/commit/49aa7cdfba19ebc6d141dca28f7ed269c2e443fa)
- [fix: crouching falling prevention does not reset velocity & fix small hitboxes horizontal collision](https://github.com/MihailRis/voxelcore/commit/bfa8b7cf3d483b2ea3f4f7166f49b1682a471363)
- [fix: macos build pipeline uses OpenAL.Framework instead of openal-soft](https://github.com/MihailRis/voxelcore/commit/cd537cc1cbf490b614cdbd8ab38a3a26364c29f6)
- [fix: assets access attempts in headless mode leading to segfaults](https://github.com/MihailRis/voxelcore/commit/a92efe581f3b6a37f3217684a7174efb50bb6ef9)
- [fix crash in headless](https://github.com/MihailRis/voxelcore/commit/2b4c3c08f35c0ce0de85789a27b77a47ce134c53)
- [fix advanced render setting tooltip](https://github.com/MihailRis/voxelcore/commit/fcbbf4814a10b6db09ba30e90b21be7dc0c96d9f)
- [fix io_stream.write_line](https://github.com/MihailRis/voxelcore/commit/4809003aa8af28ed04d78a396f32580ee5506c39)
- [fix bytearray view classes](https://github.com/MihailRis/voxelcore/commit/3f68fc1fc4fce39328a570253583d9a796779d7d)
- [fix extra faces when dense mode is off](https://github.com/MihailRis/voxelcore/commit/89485dc592f138cd244d61ac6871d745c3e82e9d)
- [fix fatal error when skeleton is missing](https://github.com/MihailRis/voxelcore/commit/8b5261f4f14ee31894dcf5125a4836ff140de8a9)
- [fix entity skeleton name serialization](https://github.com/MihailRis/voxelcore/commit/d07c8e523429393a26dbccaaf7fc7275079fa577)
- [fix on_chunk_register_event](https://github.com/MihailRis/voxelcore/commit/d197415739f0e75369a4b14de808ce9c16c808d8)
- [fix: player entity body is active before player flight state applied](https://github.com/MihailRis/voxelcore/commit/5785060ffb85edb33c72410e0c65123c094fb3dd)
- [fix: player not spawning entity after set_suspended(pid, false)](https://github.com/MihailRis/voxelcore/commit/088bfc878efc05fdd92078581068e16d1e38f652)
- [fix incorrect 3d text preset for existing players](https://github.com/MihailRis/voxelcore/commit/d9a914ee5423ec216e216f6839785a887eb47036)
- [fix: player.set_suspend(pid, true) leaves entity in the world](https://github.com/MihailRis/voxelcore/commit/a4fc4a5044d394a8f1d054ecf76d18c6ebba0a78)
- [fix nested vcm elements](https://github.com/MihailRis/voxelcore/commit/22e0c469c006217b82b1dcac9193cd38b8072cd2)
- [fix unchecked indices buffer overflow](https://github.com/MihailRis/voxelcore/commit/336ae09f669fe734ac9a69ebcbb3550596f26a56)
- [fix gui.show_input_dialog key:escape reaction](https://github.com/MihailRis/voxelcore/commit/1644751b1a32a3ecc2b4604e4a5a1cb00c7692bd)
- [fix app.reset_content args](https://github.com/MihailRis/voxelcore/commit/43fa181398c6064fd165cd6244ed4847933cef4c)
- [fix gfx.posteffects.set_intensity](https://github.com/MihailRis/voxelcore/commit/f171d3e30844c69a0675d23db583f34abdcd4f6c)
- [fix blocks raycast filter](https://github.com/MihailRis/voxelcore/commit/ce769143f881afbaa1136ec6a0692ff2b4cc2bd4)
- [fix conditional jump or move depends on uninitialised value in GLFWWindow](https://github.com/MihailRis/voxelcore/commit/e371153288c3eb99e568fe19fb725f151d6a58ac)
- [fix debugging server creation](https://github.com/MihailRis/voxelcore/commit/72040b5dbb37c6588a53e6d7584f16e98ed0a8e2)
- [fix on_blocks_tick interval](https://github.com/MihailRis/voxelcore/commit/cd43d9545a52ca153b7548585c65af8b98bbfc7f)
- [fix: inventory interaction on "disablePlayerInventory = true"](https://github.com/MihailRis/voxelcore/commit/dba10805f8d98aedf6d7ab3f5f8d7ca5c2280e8e)
- [fix: checkbox hover color](https://github.com/MihailRis/voxelcore/commit/76d5af85cc04b6560e7dde4cb9116b8d6d638036)
- [fix a typo in the vector3:rot and duplicate vector2.\_\_eq](https://github.com/MihailRis/voxelcore/commit/334666a5615e0996fa036cc44ed94777aee76867)
- [fix gfx.blockwraps.set_texture](https://github.com/MihailRis/voxelcore/commit/e423c746280c690ab26a217b1e8668253cdd1372)
- [fix: chunk border wigth change when render block selection](https://github.com/MihailRis/voxelcore/commit/963e7d78f5f555940517e9afa4dccafbe7085d17)
- [fix block model change wrapper reaction](https://github.com/MihailRis/voxelcore/commit/6d6d38c2180ae79d69498acdd3a525399579995a)
- [fix: InventoryView: when moving one object with the right mouse button, the phantom remained](https://github.com/MihailRis/voxelcore/commit/c7da5fa5698113710dca986516372a483ebfa0c7)
- [fix: integers signs are unspecified in binary_json_spec.md](https://github.com/MihailRis/voxelcore/commit/e5f9e4d02aba96678afac533be9679523487006b)
- [fix fragment:place lights](https://github.com/MihailRis/voxelcore/commit/7a513949e94764797abd09410836f4475da8156a)
- [fix: 'shadeless' block property ignored if custom model is used](https://github.com/MihailRis/voxelcore/commit/78062e2197be750d3a6fa87d44e8079286323cdd)
- [fix custom model blocks lighting](https://github.com/MihailRis/voxelcore/commit/36f7fb759a6266edae91f2e2986882eba066bdaf)
- [fix inventory.set](https://github.com/MihailRis/voxelcore/commit/c394df4e07641b9c18dc8a4ee1bcd1c519a82f66)
- [fix fatal error on invalid item id in inventory](https://github.com/MihailRis/voxelcore/commit/180dd5051b5a7358200ae705fd20b632b0d96f72)
- [fix on_block_tick timer](https://github.com/MihailRis/voxelcore/commit/77b4382577dc9cc3a21317190290e9019355ec9b)
- [fix on_block_tick triggered while paused](https://github.com/MihailRis/voxelcore/commit/4d53eb97c359b7453209af91eedfe56b03042a6e)
- [fix hover elements lifetime](https://github.com/MihailRis/voxelcore/commit/55e4e6de02e606b756dba57cefe2fdb3574c389d)
- [fix shadeless blocks with soft lights disabled](https://github.com/MihailRis/voxelcore/commit/ea8e1301eadcc1298668bcc5d7c6d9893f916e1d)
- [fix libpng error handling](https://github.com/MihailRis/voxelcore/commit/ba405ee6fc1541c2e84f62a6a047da711a313ff4)
- [fix corrupted voxel reset](https://github.com/MihailRis/voxelcore/commit/e0b5c8457b93d2caa9d7959a28960cd456bbd61d)
- [fix overriden content units scripts overriding](https://github.com/MihailRis/voxelcore/commit/719ba6e17a995d689e2c6e4845d920b6c42856c1)
- [fix block ticks](https://github.com/MihailRis/voxelcore/commit/4a880177da63b282d0889493c4497bf81c4780ae)
- [fix nil coords passed to on_block_tick](https://github.com/MihailRis/voxelcore/commit/ee21debe520a48759af9bbe2a27de148d91b371c)
- [fix: incomplete size transform support by entities](https://github.com/MihailRis/voxelcore/commit/fefc975832685f1377736aefd8a048b1fd943130)
- [fix exception if model having invalid variable texture passed to modelviewer](https://github.com/MihailRis/voxelcore/commit/1d56d83b72d59da34ceac5cc104ce3aa278dd299)
- [fix canvas:set_data with table argument](https://github.com/MihailRis/voxelcore/commit/e678195a2ae3cfd1ddc39435dd103292806dd4bc)
- [fix generated item's script](https://github.com/MihailRis/voxelcore/commit/4df4209bc59944e97102be7a618292378dc301a4)
- [fix ui elements overriding](https://github.com/MihailRis/voxelcore/commit/c214d70f1d33785f9262c552a6f7b933b41e9125)
- [fix: pack environment not passed to components](https://github.com/MihailRis/voxelcore/commit/69cf4c1c046281106571341702c48d74c27a3788)
- [fix 'unexpected end' while parsing .obj](https://github.com/MihailRis/voxelcore/commit/31e7a4ebc0076b6fdb12f4775ffba0903a11ba4e)
- [fix: bad_alloc if canvas element size is negative](https://github.com/MihailRis/voxelcore/commit/7c74eece99e1ec52e3022803975c92b951ec11c0)
