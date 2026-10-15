# Module core:bit_converter

## Converting values to bytes and back

```lua
function bit_converter.string_to_bytes(string: str) -> table
```
Converts a string to bytes

```lua
function bit_converter.bool_to_byte(boolean: bool) -> integer
```
Converts a boolean to a byte

```lua
function bit_converter.single_to_bytes(number: single) -> table
```
Converts a single precision float value to bytes

```lua
function bit_converter.double_to_bytes(number: double) -> table
```
Converts a double precision float value to bytes

```lua
function bit_converter.uint16_to_bytes(integer: int) -> table
```
Converts an unsigned 2-bytes integer to bytes

```lua
function bit_converter.uint32_to_bytes(integer: int) -> table
```
Converts an unsigned 4-bytes integer to bytes

```lua
function bit_converter.int16_to_bytes(integer: int) -> table
```
Converts a signed 2-bytes integer to bytes

```lua
function bit_converter.int32_to_bytes(integer: int) -> table
```
Converts a signed 4-bytes integer to bytes

```lua
function bit_converter.int64_to_bytes(integer: int) -> table
```
Converts a signed 8-bytes integer to bytes

```lua
function bit_converter.bytes_to_string(table: bytes) -> string
```
Converts a byte array to a string

```lua
function bit_converter.byte_to_bool(integer: byte) -> boolean
```
Converts a byte to a boolean value

```lua
function bit_converter.bytes_to_single(table: bytes) -> number№
```
Converts a byte array to a single-precision float

```lua
function bit_converter.bytes_to_double(table: bytes) -> number
```
Converts a byte array to a double precision float

```lua
function bit_converter.bytes_to_uint16(table: bytes) -> integer
```
Converts a byte array to a 2-bytes unsigned number

```lua
function bit_converter.bytes_to_uint32(table: bytes) -> integer
```
Converts a byte array to a 4-bytes unsigned number

```lua
function bit_converter.bytes_to_int16(table: bytes) -> integer
```
Converts a byte array to a 2-bytes signed number

```lua
function bit_converter.bytes_to_int32(table: bytes) -> integer
```
Converts a byte array to a 4-bytes signed number

```lua
function bit_converter.bytes_to_int64(table: bytes) -> integer
```
Converts a byte array to an 8-bytes signed number
