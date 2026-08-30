# VEC3 format specification

3D models binary format.

Byteorder: little-endian

## Syntax

```cpp
enum AttributeType:uint8 {
    POSITION = 0,
    UV,
    NORMAL,
    COLOR,
};
sizeof(AttributeType) == 1;

struct VertexAttribute {
    AttributeType type; // data type is inferred from attribute type
    uint8 flags;
    uint32 size;
    float data[]; // if compressed, first 4 bytes of compressed data is decompressed size
};
sizeof(VertexAttribute) == 6; // + dynamic data array

struct Mesh {
    uint32 triangle_count; // number of mesh triangles
    uint16 material_id;
    uint16 flags;
    uint16 attribute_count;
    VertexAttribute attributes[];
    uint8 indices[]; // if compressed, first 4 bytes of compressed data is compressed buffer size
};
sizeof(Mesh) == 10; // + dynamic attributes array + dynamic indices array

struct Model {
	uint16 name_len;
    vec3 origin;
    uint32 mesh_count;
    Mesh meshes[];
    char name[];
};
sizeof(Model) == 18; // + dynamic Mesh array + name length

struct Material {
    uint16 flags;
    uint16 name_len;
    char name[];
};
sizeof(Material) == 4; // + dynamic sized string

struct Header {
    char[8] ident;   // "\0\0VEC3\0\0"
    uint16 version;  // current is 1
    uint16 reserved; // 0x0000
};
sizeof(Header) == 12;

struct Body {
    uint16 material_count
    uint16 model_count
    Material materials[];
    Model models[];
};
sizeof(Body) == 4; // + dynamic models array + dynamic materials array

```

\* vertex data: positions are global. Model origins used to make it local.

vertex - is a set of vertex data section entries indices divided by stride, starting from 0 (section first entry).

Example: in file having sections (coordinates, texture_coordinates, normal) vertex is a set of 3 indices ordered the
same way as sections stored in the file.

## Vertex Data section tags

All sections are optional.

| Value | Name                | Stride (bytes) | Description                 |
| ----- | ------------------- | -------------- | --------------------------- |
| %x01  | Coordinates         | 12             | vertex position             |
| %x02  | Texture coordinates | 8              | vertex texture coordinates  |
| %x03  | Normals             | 12             | vertex normal vector        |
| %x04  | Color               | 16             | vertex RGBA color (0.0-1.0) |

VertexAttribute flags:

| Value | Name             |
| ----- | ---------------- |
| %x01  | ZLib compression |

## Mesh

Mesh flags:

| Value | Name                                |
| ----- | ----------------------------------- |
| %x01  | Indices ZLib compression            |
| %x02  | Use 16 bit indices instead of 8 bit |

## Material

Material flags:

| Bit offset | Description |
|------------|-------------|
| 0          | Shadeless   |
| 1-7        | Reserved    |
