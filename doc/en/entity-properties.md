# Entity properties

## Logic

### *components*

Defines components and the order in which they are initialized.

```json
"components": [
    list of components
]
```

Example:

```json
"components": [
    "base:drop"
]
```

You can pass values ​​in ARGS from the entity configuration.
They will be passed both when creating a new entity and when loading a saved one.
The `args` list is used for this:

```json
"components": [
    {
        "name": "base:drop",
        "args": {
            "item": "base:stone.item",
            "count": 1
        }
    }
]
```

The components code should be in `scripts/components`.

## Physics

### *hitbox*

An array of three numbers indicating the size of the entity's hitbox.

Example:

```json
"hitbox": [0.6, 1.8, 0.6]
```


### *body-type*

Determines how the physics engine will work with it.

- *static* - static body. No physics calculation, no velocity.
- *dynamic* - default type. The physics engine calculates movement and collisions.
- *kinematic* - only movement is calculated, without collisions.

### *blocking*

Determines whether the entity blocks installation of blocks.

*In the future will also block other entities movement.*

Default value: *true*.

### *sensors*

A sensor is an area attached to a physical body that detects the entry of other bodies into it.

- When a body enter, the *on_sensor_enter* event is triggered.
- When a body exit, the *on_sensor_exit* event is called.

Sensors are indexed in the same order as they are presented in the list, starting from 0.

The following types (shapes) of sensors exist:
- *radius* is the simplest sensor. Defines the area around the center of the hitbox. The following values ​​are specified:
    - radius - number.
- *aabb* - a rectangular area that changes position depending on the rotation of the entity. **The area itself does not rotate.** The following values ​​are specified:
    - three numbers x, y, z of the minimal corner of the area.
    - three numbers x, y, z of the opposite corner of the area.

Example:

```json
"sensors": [
    ["aabb", -0.2, -0.2, -0.2, 0.2, 0.2, 0.2],
    ["radius", 1.6]
]
```

0. A rectangular area with a width, height and length of 0.4 m, centered at 0.0.
1. Radial area with a radius of 1.6 m.

## View

### *skeleton-name*

The default value is the same as the entity name. Determines which skeleton will be used by the entity. See [rigging](rigging.md).

## Saving/Loading

In addition to custom components, the engine automatically saves data from the built-in ones: transform, rigidbody, skeleton.

There is a set of flags that allow you to specify which data will be saved and which will not.

(Boolean values ​​are specified)

| Title                  | Target                                               | Default |
| ---------------------- | ---------------------------------------------------- | ------- |
| save                   | the entity itself                                    | true    |
| save-skeleton-pose     | skeleton pose                                        | false   |
| save-skeleton-textures | dynamically assigned textures                        | false   |
| save-body-velocity     | body velocity                                        | true    |
| save-body-settings     | changed body settings <br>(type, damping, crouching) | false   |
