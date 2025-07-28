# 0.28 - 2025.07.18

[Documentation](https://github.com/MihailRis/VoxelEngine-Cpp/tree/release-0.28/doc/en/main-page.md) for 0.28

Table of contents:

- [Added](#added)
    - [Changes](#changes)
    - [Functions](#functions)
- [Fixes](#fixes)

## Added

- advanced graphics mode
- state bits based models
- post-effects
- ui elements:
    - iframe
    - select
    - modelviewer
- vcm models format
- bit.compile
- yaml encoder/decoder
- error handler argument in http.get, http.post
- ui properties:
    - image.region
- rotation profiles:
    - stairs
- libraries
    - gfx.posteffects
    - yaml
- stairs rotation profile
- models editing in console
- syntax highlighting: xml, glsl, vcm
- beginning of projects system

### Changes

- reserved 'project', 'pack', 'packid', 'root' entry points
- Bytearray optimized with FFI
- chunks non-unloading zone limited with circle 

### Functions

- yaml.tostring
- yaml.parse
- gfx.posteffects.index
- gfx.posteffects.set_effect
- gfx.posteffects.get_intensity
- gfx.posteffects.set_intensity
- gfx.posteffects.is_active
- gfx.posteffects.set_params
- gfx.posteffects.set_array
- block.get_variant
- block.set_variant
- bit.compile
- Bytearray_as_string

## Fixes

- [fix: "unknown argument --memcheck" in vctest](https://github.com/MihailRis/voxelcore/commit/281d5e09e6f1c016646af6000f6b111695c994b3)
- [fix "upgrade square is not fully inside of area" error](https://github.com/MihailRis/voxelcore/commit/bf79f6bc75a7686d59fdd0dba8b9018d6191e980 )
- [fix generator area centering](https://github.com/MihailRis/voxelcore/commit/98813472a8c25b1de93dd5d843af38c5aec9b1d8 "fix generator area centering")
- [fix incomplete content reset](https://github.com/MihailRis/voxelcore/commit/61af8ba943a24f6544c6482def2e244cf0af4d18)
- [fix stack traces](https://github.com/MihailRis/voxelcore/commit/05ddffb5c9902e237c73cdea55d4ac1e303c6a8e)
- [fix containers refreshing](https://github.com/MihailRis/voxelcore/commit/34295faca276b55c6e3c0ddd98b867a0aab3eb2a)
- [fix toml encoder](https://github.com/MihailRis/voxelcore/commit/9cd95bb0eb73521bef07f6f0d5e8b78f3e309ebf)
- [fix InputBindBox](https://github.com/MihailRis/voxelcore/commit/7c976a573b01e3fb6f43bacaab22e34037b55b73 "fix InputBindBox")
- [fix inventory.* functions error messages](https://github.com/MihailRis/voxelcore/commit/af3c315c04959eea6c11f5ae2854a6f253e3450f)
- [fix: validator not called after backspace](https://github.com/MihailRis/voxelcore/commit/df3640978d279b85653d647facb26ef15c509848)
- [fix: missing pack.has_indices if content is not loaded](https://github.com/MihailRis/voxelcore/commit/b02b45457322e1ce8f6b9735caeb5b58b1e2ffb4)
- [fix: entities despawn on F5](https://github.com/MihailRis/voxelcore/commit/6ab48fda935f3f1d97d76a833c8511522857ba6a)
- [bug fix [#549]](https://github.com/MihailRis/voxelcore/commit/49727ec02647e48323266fbf814c15f6d5632ee9)
- [fix player camera zoom with fov-effects disabled](https://github.com/MihailRis/voxelcore/commit/014ffab183687ed9acbb93ab90e43d8f82ed826a)
