# Library vec*n*

*vecn* contains a set of functions for working with vectors of dimensions 2, 3 or 4.
Most functions have several options for argument lists (overloads).

> [!WARNING]
>
> vecn, where n == vector dimension (2, 3, 4), i.e. vec2, vec3, vec4
>

## Data types

Type conventions will be used on this page.
- vector - an array of two, three or four numbers
- vec2 - array of two numbers
- vec3 - array of three numbers
- vec4 - array of four numbers

> [!WARNING]
>
> Type annotations are part of the documentation and are not specified when calling functions.


## Operations with vectors

#### Addition - *vecn.add(...)*

```lua
-- returns the result of vector addition
vecn.add(a: vector, b: vector)

-- returns the result of adding a vector and a scalar
vecn.add(a: vector, b: number)

-- writes the result of adding two vectors to dst
vecn.add(a: vector, b: vector, dst: vector)
```

#### Subtraction - *vecn.sub(...)*

```lua
-- returns the result of vector subtraction
vecn.sub(a: vector, b: vector)

-- returns the result of subtracting a scalar from a vector
vecn.sub(a: vector, b: number)

-- writes the result of subtracting two vectors to dst
vecn.sub(a: vector, b: vector, dst: vector)
```

#### Multiplication - *vecn.mul(...)*

```lua
-- returns the result of vector multiplication
vecn.mul(a: vector, b: vector)

-- returns the result of multiplying a vector by a scalar
vecn.mul(a: vector, b: number)
```

#### Inversion - *vecn.inv(...)*

```lua
-- returns the result of the inversion (opposite) of the vector
vecn.inverse(a: vector)

-- writes the inverted vector to dst
vecn.inverse(v: vector, dst: vector)
```

#### Division - *vecn.div(...)*

```lua
-- returns the result of vector division
vecn.div(a: vector, b: vector)

-- returns the result of dividing a vector by a scalar
vecn.div(a: vector, b: number)

-- writes the result of dividing two vectors to dst
vecn.div(a: vector, b: vector, dst: vector)
```

#### Normalization - *vecn.norm(...)*

```lua
-- returns normalized vector
vecn.normalize(a: vector)

-- writes the normalized vector to dst
vecn.normalize(v: vector, dst: vector)
```

#### Vector length - *vecn.len(...)*

```lua
-- returns the length of the vector
vecn.length(a: vector)

```

#### Distance - *vecn.distance(...)*

```lua
-- returns the distance between two vectors
vecn.distance(a: vector, b: vector)
```

#### Absolute value - *vecn.abs(...)*

```lua
-- returns a vector with absolute values
vecn.abs(a: vector)

-- writes the absolute value of the vector to dst
vecn.abs(v: vector, dst: vector)
```

#### Rounding - *vecn.round(...)*

```lua
-- returns a vector with rounded values
vecn.round(a: vector)

-- writes rounded vector to dst
vecn.round(v: vector, dst: vector)
```

#### Exponentiation - *vecn.pow(...)*

```lua
-- returns a vector with elements raised to powers
vecn.pow(a: vector, b: number)

-- writes the vector raised to a power to dst
vecn.pow(v: vector, exponent: number, dst: vector)
```

#### Dot product - *vecn.dot(...)*
```lua
-- returns the scalar product of vectors
vecn.dot(a: vector, b: vector)
```

#### Mixing - *vecn.mix(...)*

```lua
-- returns vector a * (1.0 - t) + b * t
vecn.mix(a: vector, b: vector, t: number)

-- writes to dst vector a * (1.0 - t) + b * t
vecn.mix(a: vector, b: vector, t: number, dst: vector)
```

#### Convert to string - *vecn.tostring(...)*
> [!WARNING]
> Returns only if the content is a vector
```lua
-- returns a string representing the contents of the vector
vecn.tostring(a: vector)
```

## Specific functions

Functions related to specific vector dimensions.

```lua
-- returns a random vector whose coordinates are uniformly distributed on a sphere of a given radius
vec3.spherical_rand(radius: number)

-- writes a random vector whose coordinates are uniformly distributed on a sphere of a given radius in dst
vec3.spherical_rand(radius: number, dst: vec3)

-- returns the direction angle of the vector v in degrees [0, 360]
vec2.angle(v: vec2)

-- returns the direction angle of the vector {x, y} in degrees [0, 360]
vec2.angle(x: number, y: number)

-- returns the vector rotated by an angle in degrees counterclockwise
vec2.rotate(v: vec2, angle: number) -> vec2

-- writes the vector rotated by an angle in degrees counterclockwise to dst
vec2.rotate(v: vec2, angle: number, dst: vec2) -> vec2
```


## Example
```lua
-- creating vectors of different dimensions
local v1_3d = {1, 2, 2}
local v2_3d = {10, 20, 40}
local v3_4d = {1, 2, 4, 1}
local v4_2d = {1, 0}
local scal = 6 -- regular scalar

-- vector addition
local result_add = vec3.add(v1_3d, v2_3d)
print("add: " .. vec3.tostring(result_add)) -- {11, 22, 42}

-- vector subtraction
local result_sub = vec3.sub(v2_3d, v1_3d)
print("sub: " .. vec3.tostring(result_sub)) -- {9, 18, 38}

-- vector multiplication
local result_mul = vec3.mul(v1_3d, v2_3d)
print("mul: " .. vec3.tostring(result_mul)) -- {10, 40, 80}

-- multiplying a vector by a scalar
local result_mul_scal = vec3.mul(v1_3d, scal)
print("mul_scal: " .. vec3.tostring(result_mul_scal)) -- {6, 12, 12}

-- calculating distance between vectors
local result_distance = vec3.distance(v1_3d, v2_3d)
print("distance: " .. result_distance) -- 43

-- vector normalization
local result_norm = vec3.normalize(v1_3d)
print("norm: " .. vec3.tostring(result_norm)) -- {0.333, 0.667, 0.667}

-- vector length
local result_len = vec3.length(v1_3d)
print("len: " .. result_len) -- 3

-- absolute value of the vector
local result_abs = vec3.abs(v1_3d)
print("abs: " .. vec3.tostring(result_abs)) -- {1, 2, 2}

-- vector rounding
local result_round = vec3.round(v1_3d)
print("round: " .. vec3.tostring(result_round)) -- {1, 2, 2}

-- vector exponentiation
local result_pow = vec3.pow(v1_3d, 2)
print("pow: " .. vec3.tostring(result_pow)) -- {1, 4, 4}

-- scalar product of vectors
local result_dot = vec3.dot(v1_3d, v2_3d)
print("dot: " ..result_dot) -- 250

-- mixing vectors
local result_mix = vec3.mix(v1_3d, v2_3d, 0.25)
print("mix: " .. vec3.tostring(result_mix)) -- {3.25, 6.5, 11.5}
