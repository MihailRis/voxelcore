# *base64* library

Library for base64 encoding/decoding.

```lua
-- Encode bytes to base64 string
base64.encode(bytes: table|ByteArray) -> str

-- Decode base64 string to ByteArray or lua table if second argument is set to true
base64.decode(base64string: str, [optional]usetable: bool=false) -> table|ByteArray

-- Encode bytes to urlsafe-base64 string ('-', '_' instead of '+', '/')
base64.encode_urlsafe(bytes: table|ByteArray) -> str

-- Decodes urlsafe-base64 string to a ByteArray or a table of numbers if the second argument is set to true
base64.decode_urlsafe(base64string: str, [optional]usetable: bool=false) -> table|ByteArray
```
