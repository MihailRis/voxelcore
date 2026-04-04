# *utf8* library

The library provides functions for working with UTF-8.

```lua
-- Converts a UTF-8 string to a Bytearray or an array of numbers if
-- the second argument is true
utf8.tobytes(text: str, [optional] usetable=false) -> Bytearray|table

-- Converts a Bytearray or an array of numbers to a UTF-8 string
utf8.tostring(bytes: Bytearray|table) -> str

-- Returns the length of a Unicode string
utf8.length(text: str) -> int

-- Returns the code of the first character of the string
utf8.codepoint(chars: str) -> int

-- Encodes codepoint in UTF-8
utf8.encode(codepoint: int) -> str

-- Returns a substring from position startchar to endchar inclusive
utf8.sub(text: str, startchar: int, [optional] endchar: int) -> str

-- Converts a string to uppercase
utf8.upper(text: str) -> str

-- Converts a string to lowercase
utf8.lower(text: str) -> str

-- Escapes a string
utf8.escape(text: str) -> str

-- Escapes special XML characters
utf8.escape_xml(text: str) -> str
```
