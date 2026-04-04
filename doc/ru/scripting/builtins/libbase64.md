# Библиотека *base64*

Библиотека для base64 кодирования/декодирования.

```lua
-- Кодирует массив байт в base64 строку
base64.encode(bytes: table|Bytearray) -> string

-- Декодирует base64 строку в Bytearray или таблицу чисел, если второй аргумент установлен на true
base64.decode(base64string: string, [опционально]usetable: boolean=false) -> table|Bytearray

-- Кодирует массив байт в urlsafe-base64 строку ('-', '_' вместо '+', '/')
base64.encode_urlsafe(bytes: table|Bytearray) -> string

-- Декодирует urlsafe-base64 строку в Bytearray или таблицу чисел, если второй аргумент установлен на true
base64.decode_urlsafe(base64string: string, [опционально]usetable: boolean=false) -> table|Bytearray
```