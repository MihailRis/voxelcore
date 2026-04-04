# Particles

Particles are a table, all fields of which are optional.

| Field           | Description                                                           | Default         |
| --------------- | --------------------------------------------------------------------- | --------------- |
| texture         | Particle texture.                                                     | ""              |
| frames          | Animation frames (array of texture names). Must be in a single atlas. | {}              |
| lighting        | Lighting.                                                             | true            |
| collision       | Collision detection.                                                  | true            |
| max_distance    | Maximum distance from the camera at which particles may spawn.        | 16.0            |
| spawn_interval  | Particle spawn interval in seconds.                                   | 1.0             |
| lifetime        | Average lifetime of particles in seconds.                             | 5.0             |
| lifetime_spread | Maximum deviation of particle lifetime (from 0.0 to 1.0).             | 0.2             |
| velocity        | Initial linear velocity of particles.                                 | {0, 0, 0}       |
| acceleration    | Particles acceleration.                                               | {0, -16, 0}     |
| explosion       | Force of particles explosion on spawn.                                | {2, 2, 2}       |
| size            | Size of particles.                                                    | {0.1, 0.1, 0.1} |
| size_spread     | Maximum particle size spread over time.                               | 0.2             |
| angle_spread    | Maximum initial rotation angle spread (0 to 1)                        | 0.0             |
| min_angular_vel | Minimum angular velocity (radians per sec). Non-negative.             | 0.0             |
| max_angular_vel | Maximum angular velocity (radians per sec). Non-negative.             | 0.0             |
| spawn_shape     | Shape of particle spawn area. (ball/sphere/box)                       | ball            |
| spawn_spread    | Size of particle spawn area.                                          | {0, 0, 0}       |
| spawn_offset    | Offset of particle spawn area.                                        | {0, 0, 0}       |
| random_sub_uv   | Size of random texture subregion (1 - entire texture will be used).   | 1.0             |

## *gfx.particles* library

```lua
gfx.particles.emit(
    -- emitter position: static coordinates or entity uid
    origin: vec3 | int,
    -- particle count (-1 - infinite)
    count: int,
    -- particle settings table
    preset: table,
    -- additional particle settings table
    [optional] extension: table
) -> int
```

Creates a particle emitter, returning its id.

```lua
gfx.particles.stop(id: int)
```

Stops the emitter permanently. The emitter will be deleted
automatically later.

```lua
gfx.particles.is_alive(id: int) -> bool
```

Checks if the emitter is running. Returns false if it is stopped or if the emitter does not exist.

```lua
gfx.particles.get_origin(id: int) -> vec3 | int
```

Returns the static position or uid of the entity the emitter is bound to.

Returns nil if the emitter does not exist.

``lua
gfx.particles.set_origin(id: int, origin: vec3 | int)
```

Sets the static position or uid of the entity the emitter will be bound to.
