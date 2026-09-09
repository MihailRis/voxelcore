# Assets preloading (*preload.json* file)

`preload.json` in content-pack folder is used for specifying additional assets should be loaded, like sounds.

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

