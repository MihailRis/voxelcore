# *pack* library

```python
pack.is_installed(packid: str) -> bool
```

Check if specified pack is installed in the world

```lua
pack.data_file(packid: str, filename: str) -> str
-- and
pack.shared_file(packid: str, filename: str) -> str
```

Returns the path to the data file
and creates missing directories in the path.

- The first option returns: `world:data/packid/filename`
- The second option returns: `config:packid/filename`

Examples:
```lua
file.write(pack.data_file(PACK_ID, "example.txt"), text)
```
For a *containermod* pack, write text to `world:data/containermod/example.txt`.

Use this to store in-world data.

```lua
file.write(pack.shared_file(PACK_ID, "example.txt"), text)
```
For a *containermod* pack, write text to `config:containermod/example.txt`

Use this to store shared data for all worlds.

```python
pack.get_folder(packid: str) -> str
```

Returns the path to the folder of the installed content pack.

Example:
```lua
file.write(pack.data_file(PACK_ID, "example.txt"), text)
```

For pack *containermod* will write text to the file `world:data/containermod/example.txt`

```python
pack.get_folder(packid: str) -> str
```

Returns installed content-pack folder.

```python
pack.is_installed(packid: str) -> bool
```

Check if the world has specified pack installed.

```python
pack.get_installed() -> strings array
```

Returns all installed content-pack ids.

```python
pack.get_available() -> strings array
```

Returns the ids of all content packs available but not installed in the world.

```python
pack.get_base_packs() -> strings array
```

Returns the id of all base packages (non-removeable)

```lua
pack.get_info(packid: str) -> {
  id: str,
  title: str,
  creator: str,
  description: str,
  version: str,
  path: str,
  icon: str, -- not available in headless mode
  dependencies: optional strings array
}
```

Returns information about the pack (not necessarily installed).
- icon - name of the preview texture (loading automatically)
- dependencies - strings following format `{lvl}{id}`, where lvl:
  - `!` - required
  - `?` - optional
  - `~` - weak
  for example `!teal`

To obtain information about multiple packs, use table of ids to avoid re-scanning:one

```lua
pack.get_info(packids: table) -> {id={...}, id2={...}, ...}
```

```lua
pack.assemble(packids: table) -> table
```

Checks the configuration for correctness and adds dependencies, returning the complete configuration.

```lua
pack.request_writeable(packid: str, callback: function(str))
```

Request pack modification permission. New entry-point will be passed to the callback if confirmed.
