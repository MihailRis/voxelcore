# *mat4* library

*mat4* contains a set of functions for work with transformation 4x4 matrices.

Most functions have several options for argument lists (overloads).

## Identity matrix - *mat4.idt(...)*

```lua
-- creates an identity matrix
mat4.idt()

-- writes an identity matrix to dst
mat4.idt(dst: matrix)
```

## Matrix determinant - *mat4.determinant(...)*

```lua
-- calculates the determinant of the matrix
mat4.determinant(m: matrix)
```

## Matrix from quaternion - *mat4.from_quat(...)*

```lua
-- creates a rotation matrix from quaternion
mat4.from_quat(quaternion: quat)

-- writes the quaternion rotation matrix to dst
mat4.from_quat(quaternion: quat, dst: matrix)
```

## Matrix multiplication - *mat4.mul(...)*

```lua
-- returns the result of matrix multiplication
mat4.mul(a: matrix, b: matrix)
-- writes the result of matrix multiplication to dst
mat4.mul(a: matrix, b: matrix, dst: matrix)

-- returns the result of multiplying a matrix and a vector
mat4.mul(a: matrix, v: vector)
-- writes the result of matrix and vector multiplication to dst
mat4.mul(a: matrix, v: vector, dst: vector)
```

## Matrix inversion - *mat4.inverse(...)*

```lua
-- returns the result of matrix inversion
mat4.inverse(m: matrix)
-- writes the result of matrix inversion to dst
mat4.inverse(m: matrix, dst: matrix)
```

## Matrix transposition - *mat4.transpose(...)*

```lua
-- returns the result of matrix transposition
mat4.transpose(m: matrix)
-- writes the result of matrix transposition to dst
mat4.transpose(m: matrix, dst: matrix)
```

## Offset - *mat4.translate(...)*

```lua
-- creates a translation matrix
mat4.translate(translation: vec3)
-- returns the result of applying a translation to matrix m
mat4.translate(m: matrix, translation: vec3)
-- writes the result of applying a translation to matrix m to dst
mat4.translate(m: matrix, translation: vec3, dst: matrix)
```
## Scaling - *mat4.scale(...)*

```lua
-- creates a scaling matrix
mat4.scale(scale: vec3)
-- returns the result of applying scaling to matrix m
mat4.scale(m: matrix, scale: vec3)
-- writes the result of applying scaling to matrix m to dst
mat4.scale(m: matrix, scale: vec3, dst: matrix)
```

## Rotation - *mat4.rotate(...)*

The angle of rotation is indicated in degrees.

```lua
-- creates a rotation matrix (angle - rotation angle) along a given axis (axis is a unit vector)
mat4.rotate(axis: vec3, angle: number)
-- returns the result of applying rotation to matrix m
mat4.rotate(m: matrix, axis: vec3, angle: number)
-- writes the result of applying rotation to matrix m to dst
mat4.rotate(m: matrix, axis: vec3, angle: number, dst: matrix)
```

## Decomposition - *mat4.decompose(...)*

Decomposes the transformation matrix into its components.

```lua
mat4.decompose(m: matrix)
-- returns a table:
{
    scale=vec3,
    rotation=matrix,
    quaternion=quat,
    translation=vec3,
    skew=vec3,
    perspective=vec4
} or nil
```

## Look at point - *mat4.look_at(...)*

```lua
-- creates a view matrix from the 'eye' point to the 'center' point with up vector specified
mat4.look_at(eye: vec3, center: vec3, up: vec3)
-- writes the view matrix to dst
mat4.look_at(eye: vec3, center: vec3, up: vec3, dst: matrix)
```

## Casting to string - *mat4.tostring(...)*

```lua
-- returns a string representing the contents of the matrix
mat4.tostring(m: matrix)
-- returns a string representing the contents of the matrix, human-readable if multiline = true
mat4.tostring(m: matrix, multiline: bool)
```
