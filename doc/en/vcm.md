# VCM

A text format for describing 3D models, replacing the deprecated `model-primitives` property.

Syntax:

```
@primitive attribute (value1, value2,...) attribute2 "value3" {
    @inner_primitive ...
}
```

Same structure in XML:

```xml
<primitive attribute="value1, value2,..." attribute2="value3">
    <inner_primitive .../>
</primitive>
```

Unlike XML, the root can contain multiple elements.

Currently, there are three types of primitives: `tri`, `box`, `rect` and `part` (describes a part of a primitive, such as the side of a cube).

> [!NOTE]
> By default, texture coordinates are determined by the dimensions of the primitives.
> For sizes greater than 1.0, the result is undefined and may change in future updates.
> Use `region` or `region-scale` for manual adjustments.

## Primitives

### `tri` Properties

This primitive describes a triangle.

- `a` - Point A. Example: `a (0,0,0)`
- `b` - Point B. Example: `b (1,0,0)`
- `c` - Point C. Example: `c (1,1,0)`
- `uv` - Normalized texture coordinates of the points (6 numbers, 2 per point). Example: `uv (0,0,1,0,1)`
- `texture` - The texture to display. Default: `$0`. Example: `texture "blocks:sand"`
- `region` - UV region within the selected texture, defined by the positions of opposite corners of a square of normalized texture coordinates. Example: `region (0,0,1,1)`
- `region-scale` - multiplier vector for the UV region. Example: `region-scale (0.5,1)`
- `shading` - determines whether shading is enabled on the primitive. Example: `shading off`

By default, texture coordinates are not affected by shape or size, unlike other primitives.

### `rect` Properties

This primitive describes a parallelogram using the right and up vectors.

- `from` - the origin of the primitive. Example: `from (0,0.5,0.125)`
- `right` - the X vector, which also determines the width of the primitive. Example: `right (1,0,0)`
- `up` - the Y vector, which also determines the primitive's height. Example: `up (0,1,0)`
- `texture` - the texture to display. Default: `$0`. Example: `texture "blocks:sand"`
- `region` - the UV region within the selected texture, defined by the positions of opposite corners. Example: `region (0,0,1,1)`
- `region-scale` - the multiplier vector for the automatically selected UV region. Example: `region-scale (0.5,1)`
- `shading` - determines whether shading is enabled on the primitive. Example: `shading off`
- `flip` - flips the UV region horizontally or vertically. Example: `flip v`

### `box` Properties

This primitive describes a rectangular parallelepiped.

- `from` - the primitive's origin. Example: `from (0,0,0)`
- `to` - the point opposite the origin. Example: `to (1,1,1)`
- `origin` - the point relative to which the rotation will be applied. Default: the center of the primitive. Example: `origin (0.5,0.5,0.5)`
- `rotate` - rotation around the (x,y,z) axes in degrees, or a quaternion (x,y,z,w). Example: `rotate (45,0,0)` or `rotate (0.3826834, 0, 0, 0.9238795)`
- `texture` - the default texture to display for all sides.
- `shading` determines whether shading is enabled on the primitive. Example: `shading off`
- `delete` deletes sides by name (top, bottom, east, west, north, south)

### `part` properties (nested in `box`)

This primitive describes the properties of the sides of a rectangular parallelepiped with the specified tags (top/bottom/north/south/east/west).

- `tags` - determines which side the properties will be applied to. Example: `tags (top,bottom)`
- `texture` - the texture to display. Default: `$side_index`.
- `region` - the UV region within the selected texture, defined by the positions of opposite corners. Example: `region (0,0,1,1)`
- `region-scale` is a multiplier vector for the automatically selected UV region. Example: `region-scale (0.5,1)`

## Skeleton

VCM can describe the [skeleton](rigging.md) of a model. The `@bone` element is used for this.

There are two types of bones:
- unnamed - used only for transformations on inner primitives and not included in the final skeleton.
- named - will be included in the final skeleton. The name is specified via the `name` attribute.

The main bone (`root`) is created automatically for the entire model.
If bones are explicitly specified, the model will be divided into several parts named as `model_name.bone_name`.
Since the model is an asset and has no prefix, the loaded skeleton receives the model name, also without a prefix.
The loaded skeleton can be used in the [entity](entity-properties.md#skeleton-name) via the `skeleton-name` property.

In the `pack/preload.json` list, if `"squash": true` is specified, the vcm will be loaded as a single model, without a skeleton.

### `bone` Properties

- `name` - The bone name. If omitted, the bone will be "dissolved" and will not be included in the final skeleton, but transformations will be applied.
- `move` - Moves nested elements.
- `scale` - Scales nested elements. Applied before moving.
- `rotate` - Rotates nested elements around the (x, y, z) axes in degrees, or as a quaternion (x, y, z, w). Example: `rotate (45,0,0)` or `rotate (0.3826834, 0, 0, 0.9238795)`
