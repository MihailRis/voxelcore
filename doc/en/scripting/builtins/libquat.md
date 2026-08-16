# *quat* library

Quaternions manipulation library.

Quaternions can be created as a table of 4 numbers:
`{w, x, y, z}`

## Quaternion from matrix - *quat.from_mat4(...)*

```lua
-- creates a quaternion based on the rotation matrix
quat.from_mat4(m: matrix, [optional] dst: quat) -> quat
```

## Quaternion of Euler angles - *quat.from_euler(...)*

```lua
-- creates quaternion from euler angles passed as XYZ (pitch, yaw, roll)
-- angles in degrees
quat.from_euler(euler: vec3, [optional] dst: quat) -> quat
```

## Quaternion composition - quat.mul(...)

```lua
-- multiplies quaternions
quat.mul(a: quat, b: quat, [optional] dst: quat) -> quat
```

## 3D vector rotation - quat.mul_vec3(...)

```lua
-- rotates vector by quaternion
quat.mul_vec3(a: quat, b: vec3, [optional] dst: vec3) -> vec3
```

## Spherical linear interpolation - *quat.slerp(...)*

The interpolation always take the short path and the rotation is performed at constant speed.

```lua
-- creates a quaternion as an interpolation between a and b,
-- where t is interpolation factor
quat.slerp(a: quat, b: quat, t: number, [optional] dst: quat) -> quat
```

## Casting to string - *quat.tostring(...)*

```lua
-- returns a string representing the contents of the quaternion
quat.tostring(q: quat) -> string
```
