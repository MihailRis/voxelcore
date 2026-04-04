# Модуль core:data_buffer

## Буффер данных
### Хранит в себе массив байтов и позволяет легко получать или добавлять разные значения

```lua
function data_buffer(
	[опционально] bytes: table,
	[опционально] order: string,
	[опционально] useBytearray: boolean
)
```
Создаёт новый экземпляр **data_buffer**.
Если **useBytearray** равен **true**, то байты буффера будут хранится ввиде **Bytearray**. Это может снизить производительность, но также и уменьшить размер буффера в памяти

```lua
function data_buffer:set_order(order: string)
```
Задаёт порядок байтов для чисел.
Должен равняться одному из перечисленных в [**bit_converter**](core_bit_converter.md)

```lua
function data_buffer:put_byte(byte: int)
```
Записывает байт в буффер

```lua
function data_buffer:put_bytes(bytes: table|Bytearray)
```
Записывает байты в буффер

```lua
function data_buffer:put_string(str: string)
```
Конвертирует строку в байты и записывает их в буффер

```lua
function data_buffer:put_bool(bool: boolean)
```
Конвертирует булевое значение в байт и записывает его в буффер

```lua
function data_buffer:put_float32(float: number)
```
Конвертирует плавающее число одинарной точности в байты и записывает их в буффер

```lua
function data_buffer:put_float64(float: number)
```
Конвертирует плавающее число двойной точности в байты и записывает их в буффер

```lua
function data_buffer:put_uint16(int: int)
```
Конвертирует беззнаковое 2-х байтовое число в байты и записывает их в буффер

```lua
function data_buffer:put_uint32(int: int)
```
Конвертирует беззнаковое 4-х байтовое число в байты и записывает их в буффер

```lua
function data_buffer:put_sint16(int: int)
```
Конвертирует знаковое 2-х байтовое число в байты и записывает их в буффер

```lua
function data_buffer:put_sint32(int: int)
```
Конвертирует знаковое 4-х байтовое число в байты и записывает их в буффер

```lua
function data_buffer:put_int64(int: int)
```
Конвертирует знаковое 8-и байтовое число в байты и записывает их в буффер

```lua
function data_buffer:put_number(num: number)
```
Конвертирует любое число в байты и записывает их в буффер;

Первый байт это тип значения:
```lua
zero = 0
uint16 = 1
uint32 = 2
int64 = 5
float64 = 6
sint16 = 7
sint32 = 8
```

```lua
function data_buffer:get_byte() -> int
```
Возвращает следующий байт из буффера

```lua
function data_buffer:get_bytes(n) -> table|Bytearray
```
Возвращает **n** следующих байтов, если **n** равен **nil** или не указан, то возвращается массив всех байтов

```lua
function data_buffer:get_string() -> string
```
Читает следующую строку из буффера

```lua
function data_buffer:get_bool() -> boolean
```
Читает следующий логический булев из буффера

```lua
function data_buffer:get_float32() -> number
```
Читает следующее плавающее число одинарной точности из буффера

```lua
function data_buffer:get_float64() -> number
```
Читает следующее плавающее число двойной точности из буффера

```lua
function data_buffer:get_uint16() -> int
```
Читает следующее 2-х байтовое беззнаковое целое число из буффера

```lua
function data_buffer:get_uint32() -> int
```
Читает следующее 4-х байтовое беззнаковое целое число из буффера

```lua
function data_buffer:get_sint16() -> int
```
Читает следующее 2-х байтовое знаковое целое число из буффера

```lua
function data_buffer:get_sint32() -> int
```
Читает следующее 4-х байтовое знаковое целое число из буффера

```lua
function data_buffer:get_int64() -> int
```
Читает следующее 8-х байтовое знаковое целое число из буффера

```lua
function data_buffer:get_number() -> number
```
Читает следующее число (см. data_buffer:put_number)

```lua
function data_buffer:size() -> int
```
Возвращает размер буффера

```lua
function data_buffer:set_position(int: pos)
```
Устанавливает текущую позицию в буффере

```lua
function data_buffer:set_bytes(table: bytes)
```
Устанавливает байты в буффер
