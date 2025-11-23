# Библиотека *base64*

Библиотека для base64 кодирования/декодирования.

```lua
-- Кодирует массив байт в base64 строку
base64.encode(bytes: table|ByteArray) -> str

-- Декодирует base64 строку в ByteArray или таблицу чисел, если второй аргумент установлен на true
base64.decode(base64string: str, [опционально]usetable: bool=false) -> table|ByteArray

-- Кодирует массив байт в urlsafe-base64 строку ('-', '_' вместо '+', '/')
base64.encode_urlsafe(bytes: table|ByteArray) -> str

-- Декодирует urlsafe-base64 строку в ByteArray или таблицу чисел, если второй аргумент установлен на true
base64.decode_urlsafe(base64string: str, [опционально]usetable: bool=false) -> table|ByteArray
```
