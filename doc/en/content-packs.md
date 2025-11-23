# Content-packs

Every content pack must have an ID following requirements:
- name can consist of Capital letters A-Z, lowercase letters a-z digits 0-9, and underscore '\_' signs. 
- the first character must not be a digit.
- name length must be in range \[2, 24\]

Content-pack folder having name same as ID may be created in *res/content*.
Content-pack folder must contain file **package.json** with following contents:

```json
{
	"id": "pack_id",
	"title": "pack name will be displayed in the content menu",
	"version": "content-pack version - major.minor",
	"creator": "content-pack creator",
	"description": "short description",
	"dependencies": [
		"pack",
		"dependencies"
	]
}
```

Dependency levels are indicated by prefixes in the name:
- '!' - required dependency
- '?' - optional dependency
- '~' - weak dependency
If prefix is not specified, '!' level will be used.

Example: '~randutil' - weak dependency 'randutil'.

Dependency version is indicated after '@' symbol and have operators to restrict acceptable versions.
If version is not specified, '\*' (any) version will be used.

Example: 'randutil@>=1.0' - dependency 'randutil' which requires version 1.0 or newer.

Example:
```json
{
    "id": "doors",
    "title": "DOORS",
    "creator": "MihailRis",
    "version": "1.0",
    "description": "doors test"
}
```

Content pack picture should be added as *icon.png* file. Recommended size: 128x128

# File System

Every loaded pack is mounted to the internal file system as a new mount point, whose name matches the pack's id.

This means that accessing pack files does not require the use of additional functions:

```lua
print(file.read("your_pack_id:package.json")) -- will output the contents of the pack's package.json
```

This is also one of the reasons why some ids are reserved and cannot be used.

Mount points are mounted as read-only. To gain write access, use the `pack.request_writeable` function.

Read more about the [file](scripting/builtins/libfile.md) library.

# Content Pack Structure

Don't be intimidated by the following text, as a minimal pack only requires `package.json`.

- Content:
    - `block_materials/` - Block material definitions
    - `blocks/` - Block definitions
    - `items/` - Item definitions
    - `generators/` - World generators
    - `entities/` - Entity definitions
    - `skeletons/` - Entity skeleton definitions
    - `presets/` - Presets (can also be used by packs for their own purposes)
        - `text3d/` - 3D Text
        - `weather/` - Weather
- Code:
    - `modules/` - Script modules
    - `scripts/` - Content scripts, world scripts
        - `components/` - Entity components
- Assets (Client-side resources):
    - `fonts/` - Fonts
    - `models/` - 3D Models
    - `textures/` - Textures
    - `shaders/` - Shaders
        - `effects/` - Post-processing effects
    - `sounds/` - Sounds and Music
    - `texts/` - Localization files
- GUI:
    - `layouts/` - UI Layouts
        - `pages/` - Menu page layouts (for the pagebox element)
- Configuration:
    - `config/` - Configuration files
        - `defaults.toml` - Overrides for standard content bindings, such as the player entity, default generator, etc.
        - `bindings.toml` - Keyboard/Mouse bindings
        - `user-props.toml` - User properties for content definitions
    - `devtools/` - Auxiliary files for internal debugging tools
    - `content.json` - Automatically generated content lists, used for validating world indices and for conversion when mismatched
    - `icon.png` - Pack icon
    - `package.json` - Pack definition file
    - `preload.json` - Asset preload lists for assets that are not loaded automatically; avoid listing assets unnecessarily
    - `resources.json` - Definition lists for [resources](resources.md) (not to be confused with assets)
    - `resource-aliases.json` - Files declaring aliases for [resources](resources.md)

> [!WARNING]
> Manually editing `content.json` is strongly discouraged and will most likely lead to irreversible world corruption.

# Content Sources

Content packs are searched for within **content sources**, which are paths in the engine's file system.

Source priority determines the scan order: if the same pack is found in multiple sources, the one belonging to the source with the highest priority will be selected.

Content sources in descending order of priority:
- `world:content` - Content in the world folder (`world/content/`)
- `user:content` - Content in the user folder (`$HOME/.voxeng/content/`)
- `project:content` - Content in the project folder (`project/content/`)
- `res:content` - Built-in content shipped with the engine core (`res/content/`)

I.e., if the same pack exists in both `world:content` and `user:content`, the version from `world:content` will be selected.

The pack version, however, is currently not taken into account.
