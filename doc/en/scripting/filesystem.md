# Filesystem and serialization

### *file* library

See [file library](builtins/libfile.md)

## *json* library

The library contains functions for serializing and deserializing tables:

```python
json.tostring(object: table, human_readable: bool=false) -> str
```

Serializes an object into a JSON string. If the second parameter is **true**, multi-line human-readable formatting will be used, rather than the compact format used by default.

```python
json.parse(code: str) -> table
```

Parses a JSON string into a table.

## *toml* library

The library contains functions for serializing and deserializing tables:

```python
toml.tostring(object: table) -> str
```

Serializes an object into a TOML string.

```python
toml.parse(code: str) -> table
```

Parses a TOML string into a table.

## *yaml* library

The library contains functions for serializing and deserializing tables:

```python
yaml.tostring(object: table) -> str
```

Serializes an object into a YAML string.

```python
yaml.parse(code: str) -> table
```

Parses a YAML string into a table.

## XML Library

The library contains functions for serializing and deserializing XML:

Further, XML as a data type denotes a Lua table with the following structure:
- Element tag is accessible by the key `#`
- XML ​​attributes are accessible as key = value pairs
- Inner elements are accessible by index

The table is used both as a dictionary and an array.

Example:

```xml
<panel size='400' color='0' interval='1' context='menu'>
    <button onclick='menu.page="worlds"'>@Worlds</button>
</panel>
```

Equivalent Lua table:

```lua
{
    ['#'] = "panel",
    size = "400",
    color = "0",
    interval = "1",
    context = "menu",
    {
        ['#'] = "button",
        onclick = 'menu.page="worlds"',
        {
            "@Worlds"
        }
    }
}
```

Consequently, the button element is accessible as `panel_element[1]`.

```lua
-- Serializes a tree in XML format
-- multiline - use multiline formatting
xml.tostring(data: XML, multiline: bool = true) -> str

-- Parses XML
xml.parse(code: str) -> XML

-- Parses the VCD (VoxelCore Declaration) format used by the VCM format.
-- root_tag - a root element with the specified tag will be implicitly created,
-- containing all root elements
xml.parse_vcd(code: str, root_tag: str = "root") -> XML
```

## *bjson* library

The library contains functions for working with the binary data exchange format [vcbjson](../../specs/binary_json_spec.md).

```lua
-- Encodes a table into a byte array
bjson.tobytes(
    -- encoded table
    value: table,
    -- compression
    [optional] compression: bool=true
) --> Bytearray

-- Decodes a byte array into a table
bjson.frombytes(bytes: table | Bytearray) --> table
```

## Storing data in a world

When saving pack data in the world, you should use the function:

```python
pack.data_file(packid: str, filename: str) -> str
```

Returns data file path like: `world:data/packid/filename`
and creates missing directories.

If paths other than `data/{packid}/...` are used, data may be lost.
