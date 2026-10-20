# Projects

A project describes the application the engine runs: its name and window title, the content packs it is built on, the permissions granted to it, and the scripts that drive it.

Content packs add content (blocks, items, generators, scripts, assets, etc.).
A project controls the application at the top level: what is loaded by default, what the main menu looks like, which engine capabilities are available.

## Project location

The project folder is selected at startup:

1. The path passed via `--project <path>`, if specified.
2. Otherwise `project/` in the user files folder (`user:project`), if it exists.
3. Otherwise `project/` in the resources folder (`res/project/`) - the default project shipped with the engine.
4. Otherwise the resources folder itself.

That is, a user can replace the default project without command line arguments by creating a `project/` folder in the user files folder (the one set via `--dir`).

```sh
VoxelCore --project path/to/project_name
```

The selected folder is mounted as the `project` entry point and is read-only for scripts:

```lua
print(file.read("project:project.toml"))
```

## Project structure

- `project_name/`
    - `project.toml` - project description
    - `start.lua` - application script (optional)
    - `project_client.lua` - client script (optional)
    - `content/` - project content packs (optional)

## Project configuration - project.toml

```toml
name = "project_name"
title = "Project Name"
base_packs = ["base"]
permissions = [
    "network",
    "record-audio",
]
```

| Field         | Type            | Description                                                                    |
| ------------- | --------------- | ------------------------------------------------------------------------------ |
| `name`        | string          | Project identifier. Used in logs.                                              |
| `title`       | string          | Shown in the window title as `<title> - VoxelCore v<version>`. Optional.       |
| `base_packs`  | list\[string\]  | Packs that are always loaded and cannot be removed in the content menu.        |
| `permissions` | list\[string\]  | Engine capabilities available to the project. See [Permissions](#permissions). |

Base packs are loaded together with the core content on every content reset (e.g., in the main menu), so their resources (textures, layouts, scripts) are available outside of worlds as well.

The default project (`res/project/project.toml`):

```toml
name = "default"
base_packs = ["base"]
permissions = [
    "network",
    "record-audio",
    "sub-instances",
]
```

### Permissions

Capabilities not listed in `permissions` are disabled for the project.

| Permission      | Grants access to                                                                                                                                      |
| --------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------- |
| `network`       | The network subsystem and the [network](scripting/builtins/libnetwork.md) library. Without it, the library functions throw an error.                  |
| `record-audio`  | Audio input devices (microphone).                                                                                                                     |
| `write-to-user` | Writing to the `user` entry point via the [file](scripting/builtins/libfile.md) library. By default only `world`, `config` and `export` are writable. |
| `sub-instances` | `app.start_background_instance` - starting background (headless) engine instances.                                                                    |
| `debugging`     | `app.start_debug_instance` - starting engine instances with a debug server.                                                                           |

> [!NOTE]
> The `--dbg-server` command line argument works even without the `network` permission.

## start.lua

`start.lua` is the **project's controlling script**. It is started after engine initialization as a coroutine
running in parallel with the main loop, and has access to the [app](scripting/builtins/libapp.md) library.

```lua
-- wait for the main menu to be ready
app.tick()

app.config_packs({"base", "my_pack"})
app.load_content()

app.new_world("demo", "42", "core:default")
```

Rules:

- The script runs in its own coroutine. `app.tick()`, `app.sleep(...)` and `app.sleep_until(...)` yield control to the engine for one or more frames.
- Content reload functions (`app.load_content`, `app.reset_content`, `app.reconfig_packs`, `app.config_packs`) must be called from this coroutine.
- The script has its own global environment: all global functions and tables are available, as well as the `app` library.
- If the engine is started with `--script <file>` or `--test <file>`, that script replaces `start.lua`: only one application script runs at a time.
- In headless mode (`--headless`) only the script passed via `--script`/`--test` runs. Without it the engine has nothing to do and exits immediately.
- In test mode (`--test`) the engine stops when the script finishes. Otherwise the engine keeps running after `start.lua` finishes.

## project_client.lua

`project_client.lua` is loaded once at startup and only in client mode (it is not loaded with `--headless`).
It controls the client-side presentation of the project - primarily screens.

The script has its own environment with access to global functions, libraries and `app`. The engine calls the following functions if they are defined:

```lua
-- Called after the screen is opened.
function on_<screen>_setup() end

-- Called before the screen is closed (when switching to another one).
function on_<screen>_clear() end
```

Screen names:

| Screen  | Description                   |
| ------- | ----------------------------- |
| `menu`  | Main menu screen.             |
| `level` | In-world screen (world open). |

When switching screens, `on_<old>_clear` is called first, then the new screen is opened, then `on_<new>_setup` is called. Errors in these functions are logged and do not stop the engine.

The engine resets the menu when the menu screen is opened; opening a page is the project's job. The default project's client script sets the menu background and opens the `main` page (simplified):

```lua
function on_menu_setup()
    setup_background()
    menu.page = "main"
    menu.visible = true
end

function on_menu_clear()
    if menubg then
        menubg:destruct()
        menubg = nil
    end
end
```

> [!NOTE]
> A project does not have to use the standard `menu`.

Pages are looked up as `layouts/pages/<name>.xml` in the core and loaded packs, so a project can ship its own main menu page in one of its base packs:

```lua
-- opens layouts/pages/title.xml from a base pack
function on_menu_setup()
    menu.page = "title"
    menu.visible = true
end
```

## project:content

The project's `content/` folder is a [content source](content-packs.md#content-sources). Packs placed in it are available to the project the same way as packs in `res/content/`:

- `my_project/`
    - `content/`
        - `pack_name/`
            - `package.json`


Content sources in descending order of priority:

- `world:content`
- `user:content`
- `project:content`
- `res:content`

## Project arguments

Arguments after a standalone `--` are passed to the project rather than the engine:

```sh
VoxelCore --project my_project -- --mode server --port 6000 --verbose
```

Each argument must have the form `--name`, optionally followed by a value. An argument without a value gets an empty string.

Project arguments are available to all scripts:

```lua
vc.get_project_arg(name: str) -> str | nil
```

```lua
local port = tonumber(vc.get_project_arg("port") or "6000")
local verbose = vc.get_project_arg("verbose") ~= nil
```

## Child instances

Engine instances started from scripts use projects too:

- `app.start_background_instance(script, output_file, project_args)` starts a headless instance with the **current project** and passes `project_args` as project arguments. Requires the `sub-instances` permission.
- `app.start_debug_instance(port, project_path, output_file)` starts an instance with a debug server. If `project_path` is specified, the instance runs that project; otherwise the project is selected the usual way. Requires the `debugging` permission.

Child instances use the same resources and user files folders as the parent. Nesting depth is limited to two levels.

See the [app library](scripting/builtins/libapp.md).
