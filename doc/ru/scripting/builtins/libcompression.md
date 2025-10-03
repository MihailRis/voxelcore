# Библиотека *compression*

Библиотека функций для работы сжатия/разжатия массивов байт

```lua
-- Сжимает массив байт.
compression.encode(
    -- Массив байт
    data: array of integers, 
    -- Алгоритм сжатия (поддерживается только gzip)
    [опционально] algorithm="gzip",
    -- Вернуть результат в table?
    [опционально] usetable=false
) -> array of integers

-- Разжимает массив байт.
compression.decode(
    -- Массив байт
    data: array of integers, 
    -- Алгоритм разжатия (поддерживается только gzip)
    [опционально] algorithm="gzip",
    -- Вернуть результат в table?
    [опционально] usetable=false
) -> array of integers
```