# Библиотека *utf8*

Библиотека предоставляет функции для работы с UTF-8.

```lua
-- Конвертирует UTF-8 строку в Bytearray или массив чисел если
-- второй аргумент - true
utf8.tobytes(text: string, [опционально] usetable: boolean=false) -> Bytearray|table

-- Конвертирует Bytearray или массив чисел в UTF-8 строку
utf8.tostring(bytes: Bytearray|table) -> string

-- Возвращает длину юникод-строки
utf8.length(text: string) -> int

-- Возвращает код первого символа строки
utf8.codepoint(chars: string) -> int

-- Кодирует код в в UTF-8
utf8.encode(codepoint: int) -> string

-- Возвращает подстроку от позиции startchar до endchar включительно
utf8.sub(text: string, startchar: int, [опционально] endchar: int) -> string

-- Переводит строку в вверхний регистр
utf8.upper(text: string) -> string

-- Переводит строку в нижний регистр
utf8.lower(text: string) -> string

-- Экранирует строку
utf8.escape(text: string) -> string

-- Экранирует спец-символы XML
utf8.escape_xml(text: string) -> string
```