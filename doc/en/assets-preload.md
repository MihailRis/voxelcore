# Assets preloading (*preload.json* file)

`preload.json` in content-pack folder is used for specifying additional assets that should be loaded, like sounds.

The file contains following categories available:
- animation
- fonts
- models
- shaders
- skeletons
- sounds
- textures

> [!NOTE]
> Sound loading with all variations following template:
> (sound: *sound_name*) -> *sound_name.ogg, sound_name_1.ogg, sound_name_2.ogg, ...*
> or *sound_name_0.ogg, sound_name_1.ogg, sound_name_2.ogg, ...*

Adding sound `packid:sounds/events/explosion.ogg` with all variants example:
```json
{
	"sounds": [
		"events/explosion"
	]
}
```

Sound will be available as "events/explosion"

Additional load settings example:
```json
{
    "sounds": [
        {
            "name": "events/explosion",
            "keep-pcm": true
        }
    ]
}
```
## Patterns

An asset name may contain `*` to include multiple assets at once. The `*` matches any sequence of characters within a single path segment.

Pattern matching is performed within the pack containing `preload.json`, specifically within the folder corresponding to the asset type (e.g., `textures/`, `sounds/`, etc.).
The resulting asset names are the same as if each discovered asset had been specified explicitly.

```json
{
    "textures": [
        "gui/*",
        "mobs/*/skin-*"
    ],
    "sounds": [
        "steps/*"
    ]
}
```

> [!NOTE]
> Sound variations (such as `grass_0.ogg`, `grass_1.ogg`, etc.) are added as a single sound entry (e.g., `steps/grass`).

A pattern ending in `/` matches directories rather than individual files. This is used for atlases:
```json
{
    "atlases": [
        "smth/*/other-*/*/"
    ]
}
```

Entry parameters apply to all matched assets. The `path` parameter cannot be used in conjunction with a pattern:
```json
{
    "sounds": [
        {
            "name": "ambient/*",
            "keep-pcm": true
        }
    ]
}
```

*preload.json* example from `core:` package (`res/preload.json`):
```json
{
    "shaders": [
        "ui3d",
        "screen",
        "background",
        "skybox_gen"
    ],
    "textures": [
        "misc/moon",
        "misc/sun",
        "gui/crosshair"
    ]
}
```

