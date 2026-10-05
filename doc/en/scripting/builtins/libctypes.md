# *ctypes* Library

A library for safe work with C types.

| ctypes.\<name\> | C-Type   | Minimum              | Maximum              |
| --------------- | -------- | -------------------- | -------------------- |
| uint8           | uint8_t  | 0                    | 255                  |
| uint16          | uint16_t | 0                    | 65535                |
| uint32          | uint32_t | 0                    | 4294967295           |
| uint64          | uint64_t | 0                    | 18446744073709551615 |
| int8            | int8_t   | -128                 | 127                  |
| int16           | int16_t  | -32768               | 32767                |
| int32           | int32_t  | -2147483648          | 2147483647           |
| int64           | int64_t  | -9223372036854775808 | 9223372036854775807  |


Usage examples:

```lua
local x = ctypes.uint64(1234)
print(x) --> 1234ULL (cdata)
local y = ctypes.uint64("18446744073709551615")
print(y) --> 18446744073709551615ULL (cdata)
print(x + y) --> 1233ULL

local z = tonumber(x)
print(z) --> 1234 (number)
```
