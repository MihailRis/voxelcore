# VCA

A text-based animation format that uses [VCM](vcm.md) syntax.

A VCA file consists of a set of directives. The order of the directives does not affect the order in which transformations are applied.

Directives are applied to the given target, which can be an entity skeleton or a camera.
When targeting a skeleton, the directive specifies the bone name via the `bone` attribute:

```vcd
@move bone bone_name ...
```

Each directive controls a single scalar value, such as x, y, z, etc., specified via the `by` attribute:

```vcd
@move bone hand by y ...
```

Two kinds of curve definitions are currently available, each producing a value `f(t) = x`, where t is time:
- [keyframe curves (curve)](#keyframe-curves)
- [expression curves (func)](#expression-curves)

## Animation directives

- `@move` - translates an object/bone
- `@rotate` - rotates an object/bone
- `@scale` - scales an object/bone (multiplier)
- `@zoom` - camera zoom (multiplier)
- `@texture` - change of a dynamically assigned texture (see [skeleton:set_texture](scripting/ecs.md#skeleton))

## Metadata

Animation metadata is set in the file via the `configure` directive:
- `fps` - frame rate, used as the divisor when calculating duration
- `frames` - number of animation frames, used as the dividend when calculating duration
- `duration` - explicitly specified duration in seconds (requires `fps` to be specified when keyframes are present)
- `rotation-order` - specifies the order in which Euler angles are applied during rotation (since the format does not take directive order into account). Examples: XYZ, ZYX, YZX.

Metadata applies globally. Its behavior does not depend on its position in the file, but placing it at the beginning is recommended.

## Additional

- `curve` - declares a custom curve type. Specify the curve name `name` and the value expression `func`.

Two keys, `kl` and `kr`, and the value `t` in the range [0..1] are available in the expression.

Example:

`@curve test-linear func (kl.value + (kr.value - kl.value) * t)`

Custom curves are referenced with a `.` prefix. Example: `@move by y curve .test-linear {...}`.

## Keyframe curves

The curve type must be specified via the `curve` attribute:
- `const` - values are not interpolated; they switch when a keyframe is reached
- `linear` - a polyline; linear interpolation is used
- `bezier` - Bézier curve keyframes with explicitly specified left and right tangents (lx, ly, rx, ry)

Keyframes are described in the following `{...}` block as ordered `@key` directives:

```vcd
@rotate by z curve linear {
    @key ...
    @key ...
}
```

A keyframe must contain a frame number `frame` and a value `value`. For `bezier` keyframes, the tangents are also specified.

Example for `curve bezier`:
```vcd
@key frame 138 value 53.00411 lx 127.067 ly 53.00411 rx 138.001 ry 53.00411
```

For the `texture` directive, it is simpler. Example:

```vcd
@texture name $0 {
    @key frame 0 value entities/tireman:face_0
    @key frame 24 value entities/tireman:face_1
}
```

> Here, `entities/tireman` is the name of the texture atlas containing `face_*` textures.
> The texture does not necessarily have to be in an atlas.

If you do not wish to limit the animation duration via `configure` but want to loop the keyframe animation for a specific line, you can use the `period` parameter to specify the animation's repetition period in frames. Example:

```vcd
@move by y curve linear period 24 {
    @key frame 0 value 0
    @key frame 12 value 1
    @key frame 24 value 0
}
```

## Expression curves

The attribute describes a function `f(t) = x`, where t is time in seconds.

Example: `func (sin(t * 2.5) + pi)`.

Operators available in expressions:

- `a + b` - addition
- `a * b` - multiplication
- `a / b` - division
- `a % b` - modulo
- `a ^ b` - exponentiation

Constants available in expressions:

- `pi` - the number π
- `e` - Euler's number

Functions available in expressions:

- `sin(x)` - sine of x
- `cos(x)` - cosine of x
- `tan(x)` - tangent of x
- `noise(x, octaves)` - fast 1D noise of x with the given number of octaves
- `noise2d(x, y, octaves)` - 2D noise of x,y with the given number of octaves
- `sign(x)` - an integer indicating the sign of x (-1/0/1)
- `rand(low, high)` - a pseudo-random floating-point number in the range from low to high
- `round(x, places)` - rounds x to the given number of decimal places `places` (optional)
- `floor(x)` - rounds x down to the nearest integer
- `ceil(x)` - rounds x up to the nearest integer
- `exp(x)` - $e^{x}$
- `min(x, ...)` - returns the smallest of the given arguments
- `max(x, ...)` - returns the largest of the given arguments
- `sqrt(x)` - square root of x
- `log(x)` - $ln({x})$
- `log10(x)` - $log_{10}({x})$
- `deg(x)` - converts radians to degrees
- `rad(x)` - converts degrees to radians


>[!WARNING]
> Using undocumented functions may have any undocumented consequences.
