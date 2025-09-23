# *app* library

A library for high-level engine control, available only in script or test mode.

The script/test name without the path and extension is available as `app.script`. The file path can be obtained as:
```lua
local filename = "script:"..app.script..".lua"
```

## Functions

```lua
app.tick()
```

Performs one tick of the main engine loop.

```lua
app.sleep(time: number)
```

Waits for the specified time in seconds, performing the main engine loop.

```lua
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

Waits for the condition checked by the function to be true, performing the main engine loop.

```lua
app.quit()
```

Terminates the engine, printing the call stack to trace the function call location.

```lua
app.reconfig_packs(
    -- packs to add
    add_packs: table,
    -- packs to remove
    remove_packs: table
)
```

Updates the packs configuration, checking its correctness (dependencies and availability of packs).
Automatically adds dependencies.

To remove all packs from the configuration, you can use `pack.get_installed()`:

```lua
app.reconfig_packs({}, pack.get_installed())
```

In this case, `base` will also be removed from the configuration.

```lua
app.config_packs(
    -- expected set of packs (excluding dependencies)
    packs: table
)
```

Updates the packs configuration, automatically removing unspecified ones, adding those missing in the previous configuration.
Uses app.reconfig_packs.

```lua
app.is_content_loaded() -> bool
```

Checks if content is loaded.

```lua
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
```

Creates a new world and opens it.

```lua
app.open_world(name: str)
```

Opens a world by name.

```lua
app.reopen_world()
```

Reopens the world.

```lua
app.save_world()
```

Saves the world.

```lua
app.close_world(
    -- save the world before closing
    [optional] save_world: bool=false
)
```

Closes the world.

```lua
app.delete_world(name: str)
```

Deletes a world by name.

```lua
app.get_version() -> int, int
```

Returns the major and minor versions of the engine.

```lua
app.get_setting(name: str) -> value
```

Returns the value of a setting. Throws an exception if the setting does not exist.

```lua
app.set_setting(name: str, value: value)
```

Sets the value of a setting. Throws an exception if the setting does not exist.

```lua
app.get_setting_info(name: str) -> {
    -- default value
    def: value,
    -- minimum value
    [only for numeric settings] min: number,
    -- maximum value
    [only for numeric settings] max: number
}
```

Returns a table with information about a setting. Throws an exception if the setting does not exist.
