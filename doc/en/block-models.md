# Block Models (depricated)

> ![WARNING]
> The `model-primitives` property is deprecated. Use the [VCM format](vcm.md) for a textual description of the model's primitives.

Block model may be created with following properties:

```js
"model": "custom",
"model-primitives": {
    "aabbs": [
	    // list of AABB primitives
    ],
    // ... other primitives
}
```

**AABB** primitive is an array of values:
```
[x, y, z, width, height, depth, texture names for all 6 sides]
```

**tetragon** primitive (more like parallelogram) an array of three vectors, describing primitive position, X vector \* width, Y vector \* height.
