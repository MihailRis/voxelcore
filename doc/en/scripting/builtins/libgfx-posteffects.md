# gfx.posteffects library

A library for post-processing effects control.

The effect slot is a resource and must be declared in resources.json in the root directory of the pack:

```json
{
    "post-effect-slot": [
        "slot_name"
    ]
}
```

```lua
-- Returns the index of the effect slot by name (pack:slot_name).
-- If the specified slot does not exist, returns -1
gfx.posteffect.index(name: str) --> int

-- Assigns the effect to the slot
gfx.posteffect.set(slot: int, effect: str)

-- Returns the effect intensity (from 0.0 to 1.0)
-- If the slot is empty, returns 0.0
gfx.posteffect.get_intensity(slot: int) --> number

-- Sets the effect intensity (from 0.0 to 1.0)
-- (The correctness of processing the parameter between 0.0 and 1.0 
-- depends on the effect)
gfx.posteffect.set_intensity(slot: int, intensity: number)

-- Returns true if the slot is not empty and the effect intensity is non-zero
gfx.posteffect.is_active(slot: int) --> bool

-- Sets parameters values ('param' directives)
gfx.posteffect.set_params(params: table)
```
