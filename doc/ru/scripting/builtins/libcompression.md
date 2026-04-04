# Библиотека *compression*

Библиотека функций для работы сжатия/разжатия массивов байт

```lua
-- Сжимает массив байт.
compression.encode(
    -- Массив байт
    data: Bytearray | table<int>, 
    -- Алгоритм сжатия (поддерживается только gzip)
    [опционально] algorithm="gzip",
    -- Вернуть результат в table?
    [опционально] usetable=false
) -> Bytearray | table<int>

-- Разжимает массив байт.
compression.decode(
    -- Массив байт
    data: Bytearray | table<int>, 
    -- Алгоритм разжатия (поддерживается только gzip)
    [опционально] algorithm="gzip",
    -- Вернуть результат в table?
    [опционально] usetable=false
) -> Bytearray | table<int>
```