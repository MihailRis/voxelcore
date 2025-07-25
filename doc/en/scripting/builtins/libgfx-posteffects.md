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
gfx.posteffects.index(name: str) --> int

-- Assigns the effect to the slot
gfx.posteffects.set_effect(slot: int, effect: str)

-- Returns the effect intensity (from 0.0 to 1.0)
-- If the slot is empty, returns 0.0
gfx.posteffects.get_intensity(slot: int) --> number

-- Sets the effect intensity (from 0.0 to 1.0)
-- (The correctness of processing the parameter between 0.0 and 1.0 
-- depends on the effect)
gfx.posteffects.set_intensity(slot: int, intensity: number)

-- Returns true if the slot is not empty and the effect intensity is non-zero
gfx.posteffects.is_active(slot: int) --> bool

-- Sets parameters values ('param' directives)
gfx.posteffects.set_params(slot: int, params: table)

-- Sets values to an array
gfx.posteffects.set_array(
    -- effect slot index
    slot: int,
    -- parameter (array) name
    name: str
    -- data string (use Bytearray_as_string function)
    data: str
)

-- Example of filling an array `#param vec3 u_arrayName[64]`:
local buffer = Bytearray(0)
for i = 0, 63 do
    local x = math.random() * 2.0 - 1.0
    local y = math.random() * 2.0 - 1.0
    local z = math.random() * 2.0 - 1.0
    Bytearray.append(buffer, byteutil.pack("fff", x, y, z))
end
gfx.posteffects.set_array(slot, "u_arrayName", Bytearray_as_string(buffer))
```
