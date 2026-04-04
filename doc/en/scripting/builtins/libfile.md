# *file* library

Filesystem interaction library.

```python
file.resolve(path: str) -> str
```

Function turns `entry_point:path` (example `user:worlds/house1`) to a regular path. (example `C://Users/user/.voxeng/worlds/house1`)

> [!NOTE]
> The function should be used for debug only. *entry_point:path* notation is required in all **file** functions.

Resulting path is not canonical and may be relative.

```python
file.read(path: str) -> str
```

Read whole text file.

```python
file.read_bytes(path: str, [optional] usetable) -> array of integers
```

Read file into bytes array. If usetable = false , returns Bytearray instead of table.

```lua
file.is_writeable(path: str) -> bool
```

Checks if the specified path is writable.

```python
file.write(path: str, text: str) -> nil
```

Overwrite text file.

```python
file.write_bytes(path: str, data: array of integers)
```

Overwrite binary file with bytes array.

```python
file.length(path: str) -> int
```

Get file length (bytes) or 0.

```python
file.exists(path: str) -> bool
```

Check if file or directory exist.

```python
file.isfile(path: str) -> bool
```

Check if the path points to a file.

```python
file.isdir(path: str) -> bool
```

Check if the path points to a directory.

```python
file.mkdir(path: str) -> bool
```

Create directory. Returns true if new directory created

```python
file.mkdirs(path: str) -> bool
```

Create directories chain. Returns true if new directory created.

```python
file.list(path: str) -> array of strings
```

Returns a list of files and directories in the specified.

```python
file.list_all_res(path: str) -> array of strings
```

Returns a list of files and directories in the specified one without specifying an entry point.

```python
file.find(path: str) -> str
```

Searches for a file from the last pack to res. The path is specified without a prefix. Returns the path with the required prefix. If the file is not found, returns nil.

```python
file.remove(path: str) -> bool
```

Deletes a file. Returns **true** if the file exists. Throws an exception on access violation.

```python
file.remove_tree(path: str) -> int
```

Recursively deletes files. Returns the number of files deleted.

```python
file.read_combined_list(path: str) -> array
```

Combines arrays from JSON files of different packs.

```python
file.read_combined_object(path: str) -> array
```

Combines objects from JSON files of different packs.

```lua
file.mount(path: str) --> str
```

Mounts a ZIP archive to the filesystem. Returns the entry point name.

```lua
file.unmount(entry_point: str) --> str
```

Unmounts the entry point.

```lua
file.create_zip(directory: str, output_file: str) --> str
```

Creates a ZIP archive from the contents of the specified directory.

```lua
file.create_memory_device() --> str
```

Creates a memory file system and returns entry point name. Lives until content unload.

```lua
file.name(path: str) --> str
```

Extracts the file name from the path. Example: `world:data/base/config.toml` -> `config.toml`.

``lua
file.stem(path: str) --> str
```

Extracts the file name from the path, removing the extension. Example: `world:data/base/config.toml` -> `config`.

```lua
file.ext(path: str) --> str
```

Extracts the extension from the path. Example: `world:data/base/config.toml` -> `toml`.

```lua
file.prefix(path: str) --> str
```

Extracts the entry point (prefix) from the path. Example: `world:data/base/config.toml` -> `world`.

```lua
file.parent(path: str) --> str
```

Returns the path one level up. Example: `world:data/base/config.toml` -> `world:data/base`

```lua
file.path(path: str) --> str
```

Removes the entry point (prefix) from the path. Example: `world:data/base/config.toml` -> `data/base/config.toml`

```lua
file.join(directory: str, path: str) --> str
```

Joins the path. Example: `file.join("world:data", "base/config.toml)` -> `world:data/base/config.toml`.

You should use this function instead of concatenating with `/`, since `prefix:/path` is not valid.
