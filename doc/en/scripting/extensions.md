# Standard Library Extensions

The **stdmin.lua** kernel script defines functions that extend and complement some of the standard **Lua** libraries.

## Contents:
- [table extensions](#table-extensions)
- [string extensions](#string-extensions)
- [math extensions](#math-extensions)
- [bit extensions](#bit-extensions)
- [additional global functions](#additional-global-functions)

## Table Extensions
```lua
-- Creates and returns a copy of the given table by creating a new one and copying all elements from the given table into it.
table.copy(t: table) -> table

-- The deep copy function creates a full copy of the source table, including all its subtables. 
table.deep_copy(t: table) -> table

-- Returns the number of pairs in the given table.
table.count_pairs(t: table) -> int

-- Returns one element from the given table at a random position.
table.random(t: table) -> any

-- Returns true if x is contained in t.
table.has(t: table, x: any) -> boolean

-- Returns the index of x in t. If the given object is not contained in the table, the function returns -1.
table.index(t: table, x: any) -> int

-- Removes element x from t.
table.remove_value(t: table, x: any)

-- Shuffles values ​​in the table.
table.shuffle(t: table) -> table

-- Adds values ​​from table t2 to table t1. If table t2 contains a key from t1, the key's value will not be changed.
table.merge(t1: table, t2: table) -> table

-- Iterates through the table and applies a func to all its elements, returning the new value of the element.
table.map(t: table, func: function(indx, value)) -> table

-- Iterates through the table using a func that returns true if the element should be kept and false if it should be deleted.
table.filter(t: table, func: function(indx, value)) -> table

-- Allows you to safely retrieve the value for the specified key. If the key exists in the table, the method will return its value.
-- If the key is missing, the method will set it to the default value and return it.
table.set_default(t: table, key: int | string, default: any) -> any

-- Returns a "flattened" version of the original table.
table.flat(t: table) -> table

-- Returns a deep "flat" version of the original table.
table.deep_flat(t: table) -> table

-- Returns a truncated version of the table from index start to index stop, inclusive. Key-value pairs
-- are not preserved in the new table. For nil values, the value starts at 1 and ends at #arr, respectively.
table.sub(arr: table, start: number | nil, stop: number | nil) -> table

-- Adds a value to the table only if it was not originally there.
table.insert_unique(t: table, val: any)
table.insert_unique(t: table, pos: int, val: any)

-- Returns a table containing all keys of the given table, including numeric ones. table.keys(t: table) -> table

-- Adds all key-value pairs from table extension to table t. If a key from t is present in extension, its value will be overwritten.
table.extend(t: table, extension: table) -> table

-- Converts the passed table to a string.
table.tostring(t: table) -> string
```

## String extensions

It's important to note that all of the functions listed below that extend **string** can be used as meta-methods on string instances, i.e.:

```lua
local str = "ABA str BAB"

if str:starts_with("ABA") and str:ends_with("BAB") then
print(str:replace("BA", "DC"))
end
```

```lua
-- Splits the string into parts based on the specified separator/expression and returns the result as a table of strings. If withpattern is true, the separator parameter will be evaluated as a regular expression. string.explode(separator: string, str: string, withpattern: boolean) -> table<string>

-- Splits the string into parts based on the specified delimiter and returns the result as a table of strings.
string.split(str: string, delimiter: string) -> table<string>

-- Escapes special characters in the string, such as `()[]+-.$%^?*`, into `%character` format. The `NUL` character (`\0`) will be converted to `%z`.
string.pattern_safe(str: string) -> string

-- Splits seconds into hours, minutes, and milliseconds and formats them using the following parameter order: `minutes, seconds, milliseconds`, and then returns the result. If format is not specified, it returns a table where:
-- h - hours,
-- m - minutes,
-- s - seconds,
-- ms - milliseconds.
string.formatted_time(seconds: number, format: string) -> string | table

-- Replaces all substrings in str equal to tofind with toreplace and returns a string with all the modified substrings.
string.replace(str: string, tofind: string, toreplace: string) -> string

-- Removes all characters equal to char from the string from the left and right ends and returns the result.
-- If the char parameter is undefined, all empty characters will be selected.
string.trim(str: string, char: string) -> string

-- Removes all characters equal to char from the string from the left end and returns the result.
-- If the char parameter is undefined, all empty characters will be selected.
string.trim_left(str: string, char: string) -> string

-- Removes all characters equal to char from the right end of the string and returns the result.
-- If the char parameter is undefined, all empty characters will be selected.
string.trim_right(str: string, char: string) -> string

-- Returns true if the string begins with the substring start.
string.starts_with(str: string, start: string) -> boolean

-- Returns true if the string ends with the substring endStr.
string.ends_with(str: string, endStr: string) -> boolean

-- The string.lower and string.upper functions are also overridden by utf8.lower and utf8.upper.

-- Escapes a string. It is an alias for utf8.escape.
string.escape(str: string) -> string

-- Escapes special XML characters. An alias for utf8.escape_xml.
string.escape_xml(text: string) -> string

-- Adds a char to the left and right of the string until its size equals size.
-- By default, char is equal to the space character.
string.pad(str: string, size: int, char: string) -> string

-- Adds a char to the left of the string until its size equals size.
-- By default, char is equal to the space character.
string.left_pad(str: string, size: int, char: string) -> string

-- Adds a char to the right of the string until its size equals size.
-- By default, char is equal to the space character.
string.right_pad(str: string, size: int, char: string) -> string

-- Encodes a string into URL format, replacing special characters with their hexadecimal representations.
string.url_encode(str: string) -> string

-- Decodes a string from URL format, replacing hexadecimal representations with their corresponding characters.
string.url_decode(str: string) -> string
```

## Math extensions

```lua
-- Returns _in if it is in the range low <= _in <= high
-- Otherwise, returns the boundary to which _in is closest.
math.clamp(_in: number, low: number, high: number) -> number

-- Returns a random fractional number in the range low to high.
math.rand(low: number, high: number) -> number

-- Returns the normalized value of num relative to conf.
math.normalize(num: number, [optional] conf: number) -> number

-- Returns the rounded value of num to the specified number of decimal places.
math.round(num: number, [optional] places: number) -> number

-- Returns the sum of all received arguments. If a table was passed as an argument, the method will return the sum of all its elements.
math.sum(x: number, ... | t: table) -> number

```

## Bit extensions
```lua

-- Common arguments:
-- * expr: A string containing a bitwise expression, conforming to the Lua 5.3 bitwise operations format
-- * args: A list of names of the expression arguments. If nil, the list is automatically generated based on the detected identifiers.

- Compiles the function to perform bitwise operations
-- * asFunction: If true, returns the function; otherwise, returns a string of function code
bit.compile(expr: string, args: table | nil, asFunction: boolean=true) -> function | string

-- Compiles the function to perform bitwise operations and executes it in place
-- * ...: Values ​​to be passed to the compiled function. bit.execute(expr: string, args: table | nil, ...) -> number
```
## Additional Global Functions

This script also defines other global functions that are available for use. Their list is below.

```lua
-- Returns true if the passed table is an array, that is, if each key is an integer greater than or equal to one
-- and if each key follows the previous one.
is_array(x: table) -> boolean

-- Splits the path into two parts and returns them: the entry point and the file path.
parse_path(path: string) -> string, string

-- Calls the function func iters times, passing it the arguments ..., and then prints to the console the time in microseconds that has elapsed
-- since the call to timeit. timeit(iters: int, func: function, ...)

-- Causes the coroutine to pause until the number of seconds specified in timesec has elapsed.
-- The function can only be used inside a coroutine.
sleep(timesec: number)

-- Waits for the passed coroutine to complete, returning the control flow. The function can only be used inside a coroutine.
-- Returns values ​​similar to those returned by pcall.
await(co: coroutine) -> result, error

-- A constant storing the PID of the current engine instance.
os.pid -> number
```
