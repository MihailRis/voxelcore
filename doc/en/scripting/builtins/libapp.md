# *app* library

A library for high-level engine control, available only in script or test mode.

The script/test name without the path and extension is available as `app.script`. The file path can be obtained as:
```lua
local filename = "script:"..app.script..".lua"
```

Since the control script may not belong to any of the packs, it does not belongs to its own package and has its own global namespace in which all global functions and tables are available, as well as the `app` library.


## Common functions

```lua
-- Performs one tick of the main engine loop.
app.tick()

-- Waits for the specified time in seconds, performing the main engine loop.
app.sleep(time: number)

-- Terminates the engine, printing the call stack
-- to trace the function call location.
app.quit()

-- Waits for the condition checked by the function to be true,
-- performing the main engine loop.
app.sleep_until(
    -- function that checks the condition for ending the wait
    predicate: function() -> bool,
    -- maximum number of engine loop ticks after which
    -- a "max ticks exceed" exception will be thrown
    [optional] max_ticks = 1e9,
    -- maximum wait time in seconds.
    -- (works with system time, including test mode)
    [optional] timeout = 1e9
)
```

## Content

```lua
-- Checks if content is loaded.
app.is_content_loaded() -> bool

-- Returns the current content configuration
-- (list of packs IDs in loading order)
app.get_content() -> table<string>

-- Loads content based on the current configuration.
-- Cannot be used if content is already loaded (see app.reset_content).
app.load_content()

-- Unloads all content, resetting it to a single core pack.
app.reset_content(
    -- Packs for which modules, events, and environment will not be reset
    [optional] non_reset_packs: table
)

-- Updates the packs configuration, checking its correctness
-- (dependencies and availability of packs).
-- Automatically adds and reorders packs based on dependencies.
app.reconfig_packs(
    -- packs to add
    add_packs: table,
    -- packs to remove
    remove_packs: table
)

-- To remove all packs from the configuration,you can use `pack.get_installed()`:
app.reconfig_packs({}, pack.get_installed())
-- In this case, `base` will also be removed from the configuration.

-- Updates the packs configuration, automatically removing unspecified ones,
-- adding those missing in the previous configuration.
-- Uses app.reconfig_packs.
app.config_packs(
    -- expected set of packs (excluding dependencies)
    packs: table
)
```

## Worlds


```lua
-- Creates a new world and opens it.
app.new_world(
    -- world name, empty string will create a nameless world
    name: str,
    -- generation seed
    seed: str,
    -- generator name
    generator: str
    -- local player id
    [optional] local_player: int=0
)

-- Deletes a world by name.
app.delete_world(name: str)

-- Opens a world by name.
app.open_world(name: str)

-- Reopens the world.
app.reopen_world()

-- Saves the world.
app.save_world()

-- Closes the world.
app.close_world(
    -- save the world before closing
    [optional] save_world: bool=false
)
```

## Engine settings and information

```lua
-- Returns the major and minor versions of the engine.
app.get_version() -> int, int

-- Returns the value of a setting.
-- Throws an exception if the setting does not exist.
app.get_setting(name: str) -> value

-- Sets the value of a setting.
-- Throws an exception if the setting does not exist.
app.set_setting(name: str, value: value)

-- Returns a table with information about a setting.
-- Throws an exception if the setting does not exist.
app.get_setting_info(name: str) -> {
    -- default value
    def: value,
    -- minimum value
    [only for numeric settings] min: number,
    -- maximum value
    [only for numeric settings] max: number
}

-- Brings the window to front and sets input focus.
app.focus()
```

## Paths and entry points

```lua
-- Creates an in-memory filesystem.
app.create_memory_device(
    -- entry-point name
    name: str
)

-- Returns a list of content sources (paths), in descending priority order.
app.get_content_sources() -> table<string>

-- Sets a list of content sources (paths). Specified in descending priority order.
app.set_content_sources(sources: table<string>)

-- Resets content sources.
app.reset_content_sources()
```

## Sub-instances

```lua
-- Creates a headless engine instance with the current project and the specified application script.
-- Returns the instance ID. The number of active sub-instances is currently limited to one.
app.start_background_instance(
    -- script file
    app_script: string,
    -- log file
    output_file: string,
    -- project parameters (can be read with vc.get_project_arg(name))
    project_args: table<string, string> | nil
) -> int

-- Checks if the engine sub-instance is alive.
app.is_instance_alive(handle: int) -> boolean

-- Stops the engine sub-instance.
-- Returns true if the instance was alive at the time of the call.
app.terminate_instance(handle: int) -> boolean
```
