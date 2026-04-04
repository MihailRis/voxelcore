# Library *entities*

The library is designed to work with a registry of entities.

```lua
-- Returns an entity by unique identifier
-- The table returned is the same one available in the entity components.
entities.get(uid: int) -> table

-- Creates the specified entity.
-- args - table of component parameter tables (ARGS variable)
-- args is optional
-- args structure:
-- {prefix__name={...}, ...}
-- prefix - component pack id
-- name - component name
-- component prefix and name are separated with two underscores
-- Returns an entity object
entities.spawn(name: str, pos: vec3, [optional] args: table) -> table

-- Checks the existence of an entity by a unique identifier.
entities.exists(uid: int) -> bool

-- Returns entity definition index by UID
entities.get_def(uid: int) -> int

-- Returns entity 'hitbox' property value
entities.def_hitbox(id: int) -> vec3

-- Returns entity definition name by index (string ID).
entities.def_name(id: int) -> str

-- Returns entity definition index by name (integer ID).
entities.def_index(name: str) -> int

-- Returns number of available entity definitions
entities.defs_count() -> int

```

> [!NOTE]
> The following functions for retrieving multiple entities return tables with integer keys (uid).
> For iteration, use `pairs`, and for calculating the table size, use `table.count_pairs`.
> Using `ipairs` will yield incorrect results, since the returned table is not an array.

```lua

-- Returns a table of all loaded entities
entities.get_all() -> table

-- Returns a table of loaded entities based on the passed list of UIDs
entities.get_all(uids: array<int>) -> table

-- Returns a list of UIDs of entities inside the rectangular area
-- pos - minimal area corner
-- size - area size
entities.get_all_in_box(pos: vec3, size: vec3) -> array<int>

-- Returns a list of UIDs of entities inside the radius
-- center - center of the area
-- radius - radius of the area
entities.get_all_in_radius(center: vec3, radius: number) -> array<int>
```

```lua
entities.raycast(start: vec3, dir: vec3, max_distance: number, ignore: int, [optional] destination: table,
    [optional] filter: table, [optional] include_non_selectable = false
) -> table or nil
```

The function is an extended version of [block.raycast](libblock.md#raycast). Returns a table with the results if the ray touches a block or entity.

Accordingly, this will affect the presence of the *entity* and *block* fields.
