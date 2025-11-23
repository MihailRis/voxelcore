# gfx.skeletons library

A library for working with named skeletons, such as 'hand',
used to control the hand and the carried item displayed in first-person view. 
The set of functions is similar to the skeleton component of entities.

The first argument to the function is the name of the skeleton.

```lua
-- Returns an object wrapper over the skeleton
local skeleton = gfx.skeletons.get(name: str)

-- Returns the index of the bone by name or nil
skeleton:index(name: str) -> int

-- Returns the name of the model assigned to the bone with the specified index
skeleton:get_model(index: int) -> str

-- Reassigns the model of the bone with the specified index
-- Resets to the original if you do not specify a name
skeleton:set_model(index: int, name: str)

-- Returns the transformation matrix of the bone with the specified index
skeleton:get_matrix(index: int) -> mat4

-- Sets the transformation matrix of the bone with the specified index
skeleton:set_matrix(index: int, matrix: mat4)

-- Returns the texture by key (dynamically assigned textures - '$name')
skeleton:get_texture(key: str) -> str

-- Assigns a texture by key
skeleton:set_texture(key: str, value: str)

-- Checks the visibility status of a bone by index
-- or the entire skeleton if index is not specified
skeleton:is_visible([optional] index: int) -> bool

-- Sets the visibility status of a bone by index
-- or the entire skeleton if index is not specified
skeleton:set_visible([optional] index: int, status: bool)

-- Returns the color of the entity
skeleton:get_color() -> vec3

-- Sets the color of the entity
skeleton:set_color(color: vec3)
```
