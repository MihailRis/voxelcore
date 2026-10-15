# Engine usage recommendations

## Content naming

### Content packs ID

Content pack identifier requirements:
- name can consist of Capital letters A-Z, lowercase letters a-z, digits 0-9, and underscore '\_' signs. 
- the first character must not be a digit.
- name length must be in range \[2, 24\]

### Blocks and items

- blocks and items identifiers follow the same requirements as content-pack ID.
- `.item` suffix added only to replace auto-generated block item. Example: `base:stone.item` - an item generated for stone block.
- **caption** field specifying name displayed in inventory UI should not be Capitalized. The engine does it automatically depending on display context.

## Storage

### Content packs data

State that is supposed to be saved with a world, must be stored in `world:data/pack_id/`. The path should be retrieved by calling a function:

```lua
local path = pack.data_file(PACK_ID, "file_name")
file.write(path, some_data)
-- writes data to file world:data/PACK_ID/file_name
```
PACK_ID is an existing variable containing current content-pack name.

Directory `world:data/PACK_ID` will be created on call `pack.data_file(...)`.

#### Shared data

Settings and other data that should be accessible from all worlds where the pack is used should be in
`config:pack_id/`. You can use a special function:

```lua
local path = pack.shared_file(PACK_ID, "file_name")
file.write(path, data)
-- writes data to the file config:PACK_ID/file_name
```
