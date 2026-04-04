# Модуль core:bit_converter

## Доступные порядки байтов
**LE (Little-Endian)**  
**BE (Big-Endian)**
По умолчанию используется **LE**

## Конвертация значений в байты и обратно

```lua
function bit_converter.string_to_bytes(str: string) -> table
```
Конвертирует строку в байты

```lua
function bit_converter.bool_to_byte(bool: boolean) -> int
```
Конвертирует логический булев в байт

```lua
function bit_converter.float32_to_bytes(float: number, [опционально] order: string) -> table
```
Конвертирует плавающее значение одинарной точности в байты

```lua
function bit_converter.float64_to_bytes(float: number, [опционально] order: string) -> table
```
Конвертирует плавающее значение двойной точности в байты

```lua
function bit_converter.uint16_to_bytes(int: int, [опционально] order: string) -> table
```
Конвертирует беззнаковое 2-х байтовое целое число в байты

```lua
function bit_converter.uint32_to_bytes(int: int, [опционально] order: string) -> table
```
Конвертирует беззнаковое 4-х байтовое целое число в байты

```lua
function bit_converter.sint16_to_bytes(int: int, [опционально] order: string) -> table
```
Конвертирует знаковое 2-х байтовое целое число в байты

```lua
function bit_converter.sint32_to_bytes(int: int, [опционально] order: string) -> table
```
Конвертирует знаковое 4-х байтовое целое число в байты

```lua
function bit_converter.int64_to_bytes(int: int, [опционально] order: string) -> table
```
Конвертирует знаковое 8-и байтовое целое число в байты

```lua
function bit_converter.bytes_to_string(bytes: table) -> string
```
Конвертирует массив байтов в строку

```lua
function bit_converter.byte_to_bool(byte: int) -> boolean
```
Конвертирует байт в логическое булевое значение

```lua
function bit_converter.bytes_to_float32(bytes: table|Bytearray, [опционально] order: string) -> number
```
Конвертирует массив байтов в плавающее число одинарной точности

```lua
function bit_converter.bytes_to_float64(bytes: table|Bytearray, [опционально] order: string) -> number
```
Конвертирует массив байтов в плавающее число двойной точности

```lua
function bit_converter.bytes_to_uint16(bytes: table|Bytearray, [опционально] order: string) -> int
```
Конвертирует массив байтов в 2-х байтовое беззнаковое число

```lua
function bit_converter.bytes_to_uint32(bytes: table|Bytearray, [опционально] order: string) -> int
```
Конвертирует массив байтов в 4-х байтовое беззнаковое число

```lua
function bit_converter.bytes_to_sint16(bytes: table|Bytearray, [опционально] order: string) -> int
```
Конвертирует массив байтов в 2-х байтовое знаковое число

```lua
function bit_converter.bytes_to_sint32(bytes: table|Bytearray, [опционально] order: string) -> int
```
Конвертирует массив байтов в 4-х байтовое знаковое число

```lua
function bit_converter.bytes_to_int64(bytes: table|Bytearray, [опционально] order: string) -> int
```
Конвертирует массив байтов в 8-х байтовое знаковое число
