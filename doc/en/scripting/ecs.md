# Entities and components

## Entity

The entity object is available in components as a global variable **entity**.

```lua
-- Deletes an entity (the entity may continue to exist until the frame ends, but will not be displayed in that frame)
entity:despawn()

-- Returns entity defintion index (integer ID)
entity:def_index() -> int

-- Returns entity definition name (string ID)
entity:def_name() -> str

-- Returns the name of the entity skeleton
entity:get_skeleton() -> str
-- Replaces the entity skeleton
entity:set_skeleton(name: str)

-- Returns the unique entity identifier
entity:get_uid() -> int

-- Returns the component by name
entity:get_component(name: str) -> component or nil
-- Checks for the presence of a component by name
entity:has_component(name: str) -> bool
-- Retrieves a component by name. Throws an exception if it does not exist
entity:require_component(name: str) -> component

-- Enables/disables the component
entity:set_enabled(name: str, enable: bool)

-- Returns id of player the entity is bound
entity:get_player() -> int or nil
```

## Built-in components

### Transform

The component is responsible for the position, scale and rotation of the entity.

```lua
-- Alias
local tsf = entity.transform

-- Returns the position of the entity
tsf:get_pos() -> vec3
-- Sets the entity position
tsf:set_pos(pos:vec3)

-- Returns the entity scale 
tsf:get_size() -> vec3
-- Sets the entity scale
tsf:set_size(size: vec3)

-- Returns the entity rotation
tsf:get_rot() -> mat4
-- Sets entity rotation
tsf:set_rot(rotation: mat4)
```

### Rigidbody

The component is responsible for the physical body of the entity.

```lua
-- Alias
local body = entity.rigidbody

-- Checks if body physics calculation is enabled
body:is_enabled() -> bool
-- Enables/disables body physics calculation
body:set_enabled(enabled: bool)

-- Returns linear velocity
body:get_vel() -> vec3
-- Sets linear velocity
body:set_vel(vel: vec3)

-- Returns the size of the hitbox
body:get_size() -> vec3
-- Sets the hitbox size 
body:set_size(size: vec3)

-- Returns the gravity multiplier
body:get_gravity_scale() -> vec3
-- Sets the gravity multiplier
body:set_gravity_scale(scale: vec3)

-- Returns the linear velocity attenuation multiplier (used to simulate air resistance and friction)
body:get_linear_damping() -> number
-- Sets the linear velocity attenuation multiplier
body:set_linear_damping(value: number)

-- Checks if vertical damping is enabled
body:is_vdamping() -> bool
-- Returns the vertical damping multiplier
body:get_vdamping() -> number
-- Enables/disables vertical damping / sets vertical damping multiplier
body:set_vdamping(enabled: bool | number)

-- Checks if the entity is on the ground
body:is_grounded() -> bool

-- Checks if the entity is in a "crouching" state (cannot fall from blocks)
body:is_crouching() -> bool
-- Enables/disables the "crouching" state
body:set_crouching(enabled: bool)

-- Returns the type of physical body (static/dynamic/kinematic)
body:get_body_type() -> str
-- Sets the physical body type
body:set_body_type(type: str)
```

### Skeleton

The component is responsible for the entity skeleton. See [rigging](../rigging.md).

```lua
-- Alias
local rig = entity.skeleton

-- Returns the model name assigned to the bone at the specified index
rig:get_model(index: int) -> str

-- Reassigns the bone model at the specified index
-- Resets to original if name is not specified
rig:set_model(index: int, name: str)

-- Returns the bone transformation matrix at the specified index
rig:get_matrix(index: int) -> mat4
-- Sets the bone transformation matrix at the specified index
rig:set_matrix(index: int, matrix: mat4)

-- Returns the texture by key (dynamically assigned textures - '$name')
rig:get_texture(key: str) -> str

-- Assigns texture by key
rig:set_texture(key: str, value: str)

-- Returns the bone index by name or nil
rig:index(name: str) -> int

-- Checks the visibility status of a bone by index 
-- or the skeleton if no index is specified
rig:is_visible([optional] index: int) -> bool

-- Sets the visibility status of a bone by index
-- or the skeleton if no index is specified
rig:set_visible([optional] index: int, status: bool)

-- Returns the color of the entity
rig:get_color() -> vec3

-- Sets the color of the entity
rig:set_color(color: vec3)
```

## Component events

```lua
function on_despawn()
```

Called when the entity is despawned.

```lua
function on_grounded(force: number)
```

Called on landing. The first argument is the impact force (Speed module).

```lua
function on_fall()
```

Called when the entity starts to fall.

```lua
function on_save()
```

Called before component data is saved. Here you can write the data you want to save into the *SAVED_DATA* table, which is available for the entire life of the component.

```lua
function on_update(tps: int)
```

Called every entities tick (currently 20 times per second).

```lua
function on_physics_update(delta: number)
```

Called after each physics step

```lua
function on_render(delta: number)
```

Called every frame before the entity is rendered.

```lua
function on_sensor_enter(index: int, entity: int)
```

Called when another entity hits the sensor with the index passed as the first argument. The UID of the entity that entered the sensor is passed as the second argument.

```lua
function on_sensor_exit(index: int, entity: int)
```

Called when another entity exits the sensor with the index passed as the first argument. The UID of the entity that left the sensor is passed as the second argument.

```lua
function on_aim_on(playerid: int)
```

Called when the player aims at the entity. The player ID is passed as an argument.

```lua
function on_aim_off(playerid: int)
```

Called when the player takes aim away from the entity. The player ID is passed as an argument.

```lua
function on_attacked(attackerid: int, playerid: int)
```

Called when an entity is attacked (LMB on the entity). The first argument is the UID of the attacking entity. The attacking player ID is passed as the second argument. If the entity was not attacked by a player, the value of the second argument will be -1.


```lua
function on_used(playerid: int)
```

Called when an entity is used (RMB by entity). The player ID is passed as an argument.
