# *byteutil* library

The library provides functions for working with byte arrays represented as tables or Bytearrays.

```lua
byteutil.pack(format: str, ...) -> Bytearray
byteutil.tpack(format: str, ...) -> table
```

Returns a byte array containing the provided values packed according to the format string. The arguments must exactly match the values required by the format.

The format string consists of special characters and value characters.

Special characters specify the byte order for the subsequent values:

| Character | Byte Order          |
| --------- | ------------------- |
| `@`       | System              |
| `=`       | System              |
| `<`       | Little-endian       |
| `>`       | Big-endian          |
| `!`       | Network (big-endian)|


Value characters describe the type and size.

| Character | C++ Equivalent  | Lua Type | Size    |
| --------- | --------------- | -------- | ------- |
| `b`       | int8_t          | number   | 1 byte  |
| `B`       | uint8_t         | number   | 1 byte  |
| `?`       | bool            | boolean  | 1 byte  |
| `h`       | int16_t         | number   | 2 bytes |
| `H`       | uint16_t        | number   | 2 bytes |
| `i`       | int32_t         | number   | 4 bytes |
| `I`       | uint32_t        | number   | 4 bytes |
| `f`       | float32_t       | number   | 4 bytes |
| `l`       | int64_t         | number   | 8 bytes |
| `L`       | uint64_t        | number   | 8 bytes |
| `d`       | float64_t       | number   | 8 bytes |

> [!WARNING]
> Due to the absence of an integer type in Lua for values `l` and `L`, only an output size of 8 bytes is guaranteed; the value may differ from what is expected.

```lua
byteutil.unpack(format: str, bytes: table|Bytearray) -> ...
```

Extracts values ​​from a byte array based on a format string.

Example:

```lua
debug.print(byteutil.tpack('>iBH?', -8, 250, 2019, true))
-- outputs:
-- debug.print(
--  {
--    255,
--    255,
--    255,
--    248,
--    250,
--    7,
--    227,
--    1
--  }
-- )

local bytes = byteutil.pack('>iBH?', -8, 250, 2019, true)
print(byteutil.unpack('>iBH?', bytes))
-- outputs:
--  -8      250     2019    true
```
