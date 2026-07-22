# *world* library

```lua
-- Checks if world is open
world.is_open() -> bool

-- Returns worlds information.
world.get_list() -> tables array {
    -- world name
    name: str,
    -- world icon/preview (loading automatically)
    icon: str
    -- engine version the world was saved on
    version: {int, int}
}

-- Returns current day time in range \[0.0-1.0\] 
-- where 0.0 and 1.0 - midnight, 0.5 - noon.
world.get_day_time() -> number

-- Set day time value.
world.set_day_time(time: number)

-- Sets the specified day time cycle speed.
world.set_day_time_speed(value: number)

-- Returns the day time cycle speed.
world.get_day_time_speed() -> number

-- Returns total time passed in the world.
world.get_total_time() -> number

-- Returns world seed.
world.get_seed() -> int

-- Returns generator name.
world.get_generator() -> str

-- Checks the existence of a world by name.
world.exists(name: str) -> bool

-- Checks if the current time is daytime. From 0.333(8am) to 0.833(8pm).
world.is_day() -> bool

-- Checks if the current time is nighttime. From 0.833(8pm) to 0.333(8am).
world.is_night() -> bool

-- Returns the total number of chunks loaded into memory
world.count_chunks() -> int

-- Returns the compressed chunk data to send.
-- If the chunk is not loaded, returns the saved data.
-- Currently includes:
-- 1. Voxel data (id and state)
-- 2. Voxel metadata (fields)
world.get_chunk_data(x: int, z: int) -> Bytearray or nil

-- Modifies the chunk based on the compressed data.
-- Returns true if the chunk exists.
world.set_chunk_data(
    x: int, z: int,
    -- compressed chunk data
    data: Bytearray
) -> bool

-- Saves chunk data to region.
-- Changes will be written to file only on world save.
world.save_chunk_data(
    x: int, z: int,
    -- compressed chunk data
    data: Bytearray
)

-- Casts a ray from the start point in the dir direction.
world.raycast(
    params: table {
        [==[ required parameters ]==]
        -- starting point
        start: vec3,
        -- ray direction vector
        dir: vec3,
        -- maximum distance
        distance: number,

        [==[ additional parameters (entities) ]==]
        -- consider entities
        entities: bool = true,
        -- uid of the ignored entity
        ignore_uid: number = 0 (ENTITY_NONE),
        -- id of the entity types used for filtering
        filter_entities: table<string|number> = nil,
        -- exclusion filtering mode:
        --   if true, filter_entities defines the list of ignored entity types
        entities_exclusion: bool = false,

        [==[ additional parameters (blocks) ]==]
        -- id of the block types used for filtering
        filter_blocks: table<string|number> = nil,
        -- exclusion filtering mode:
        --   if true, filter_blocks defines the list Ignored
        blocks_exclusion: bool = true,
        -- include blocks with selectable=false
        nonselect_blocks: bool = false;
    }
) -> {
    block: int or nil, -- block id
    entity: int or nil, -- entity uid
    endpoint: vec3, -- ray touchpoint
    iendpoint: vec3, -- position of the block touched by the ray
    length: number, -- ray length
    normal: vec3, -- normal vector of the surface touched by the ray
} or nil if the ray did not touch a block or entity
```
```
