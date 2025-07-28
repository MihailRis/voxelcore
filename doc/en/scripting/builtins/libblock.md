# *block* library

```lua
-- Returns block string ID (name) by index.
block.name(blockid: int) -> str

-- Returns block integer ID (index) by name.
block.index(name: str) -> int

-- Returns the id of the block material.
block.material(blockid: int) -> str

-- Table of materials by their full names (example: base:carpet)
block.materials: table<string, table>

-- Table of custom block properties (see ../../block-properties.md)
block.properties: table<int, table<string, object>>

-- Returns the block display name.
block.caption(blockid: int) -> str

-- Returns integer ID by block position
-- If the chunk at the specified coordinates is not loaded, returns -1.
block.get(x: int, y: int, z: int) -> int

-- Returns block state (rotation + additional information) as an integer.
-- Used to save complete block information.
block.get_states(x: int, y: int, z: int) -> int

-- Set block with given integer ID and state (default - 0) at given position.
block.set(x: int, y: int, z: int, id: int, states: int)

-- Places a block with a given integer id and state (default - 0) at given position.
-- on behalf of the player, calling the on_placed event.
-- playerid is optional
block.place(x: int, y: int, z: int, id: int, states: int, [optional] playerid: int)

-- Breaks a block at the given coordinates on behalf of the player, triggering the on_broken event.
-- playerid is optional
block.destruct(x: int, y: int, z: int, playerid: int)

-- Compose the complete state as an integer
block.compose_state(state: {rotation: int, segment: int, userbits: int}) -> int

-- Decompose the complete state into: rotation, segment, user bits
block.decompose_state(state: int) -> {int, int, int}
```

> [!WARNING]
> `block.set` does not trigger on_placed.

```lua
-- Check if block at the specified position is solid.
block.is_solid_at(x: int, y: int, z: int) -> bool

-- Check if block may be placed at specified position.
-- (Examples: air, water, grass, flower)
block.is_replaceable_at(x: int, y: int, z: int) -> bool

-- Returns count of available block IDs.
block.defs_count() -> int

-- Returns the index of the item specified in the *picking-item* property.
block.get_picking_item(id: int) -> int

-- Returns the block variant index
block.get_variant(x: int, y: int, z: int) -> int

-- Sets the block variant by index
block.set_variant(x: int, y: int, z: int, index: int) -> int
```

## Rotation

Following three functions return direction vectors based on block rotation.


```lua
-- Returns X: integer direction vector of the block at specified coordinates.
-- Example: no rotation: 1, 0, 0.
block.get_X(x: int, y: int, z: int) -> int, int, int
block.get_X(id: int, rotation: int) -> int, int, int

-- Same for axis Y. Default: 0, 1, 0.
block.get_Y(x: int, y: int, z: int) -> int, int, int
block.get_Y(id: int, rotation: int) -> int, int, int

-- Same for axis Z. Default: 0, 0, 1.
block.get_Z(x: int, y: int, z: int) -> int, int, int
block.get_Z(id: int, rotation: int) -> int, int, int

-- Returns block rotation index based on used profile.
block.get_rotation(x: int, y: int, z: int) -> int

-- Set block rotation by index.
block.set_rotation(x: int, y: int, z: int, rotation: int)
```

## Extended blocks

Extended blocks are blocks with size greather than 1x1x1

```lua
-- Checks whether the block is extended.
block.is_extended(id: int) -> bool

-- Returns the block size.
block.get_size(id: int) -> int, int, int

-- Checks whether the block is a non-origin segment of an extended block.
block.is_segment(x: int, y: int, z: int) -> bool

-- Returns the position of the main segment of an extended block
-- or the original position, if the block is not extended.
block.seek_origin(x: int, y: int, z: int) -> int, int, int
```

## User bits

Part of a voxel data used for scripting. Size: 8 bit.

```python
block.get_user_bits(x: int, y: int, z: int, offset: int, bits: int) -> int
``` 

Get specified bits as an unsigned integer.

```python
block.set_user_bits(x: int, y: int, z: int, offset: int, bits: int, value: int) -> int
```
Set specified bits.

## Raycast

```lua
block.raycast(start: vec3, dir: vec3, max_distance: number, [optional] dest: table, [optional] filter: table) -> {
    block: int, -- block id
    endpoint: vec3, -- point of the ray hit point
    iendpoint: vec3, -- position of the block hit by the ray
    length: number, -- ray length
    normal: vec3, -- normal vector of the surface hit by the ray
} or nil
```

Casts a ray from the start point in the direction of *dir*. Max_distance specifies the maximum ray length.

Argument `filter` can be used to tell ray what blocks can be skipped(passed through) during ray-casting.
To use filter `dest` argument must be filled with some value(can be nil), it's done for backwards compatability 

The function returns a table with the results or nil if the ray does not hit any block.

The result will use the destination table instead of creating a new one if the optional argument specified.

## Data fields

```lua
-- writes a value to the specified block field
-- * throws an exception if the types are incompatible
-- * throws an exception when array is out of bounds
-- * does nothing if the block does not have the field
block.set_field(
    x: int, y: int, z: int,
    name: str,
    value: bool|int|number|string,
    [optional] index: int = 0
)

-- returns the value written to the block field
-- * returns nil if:
--     1. the field does not exist
--     2. no writes were made to any block field
-- * throws an exception when array is out of bounds
block.get_field(
    x: int, y: int, z: int,
    name: str,
    [optional] index: int = 0
) -> the stored value or nil
```
