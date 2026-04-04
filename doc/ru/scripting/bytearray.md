# Класс *Bytearray*

*Bytearray* - динамический байтовый массив, реализованный через LuaJIT FFI. По нему можно итерироваться стандартными функциями **pairs** и **ipairs**

## Основное
### Создание массива
```lua
local bytes = Bytearray()           -- пустой
local bytes = Bytearray("hello")    -- из строки
local bytes = Bytearray({1,2,3})    -- из таблицы чисел
```

### Методы
```lua
-- Добавляет элемент(ы) в конец массива.
bytes:append(value: int)
bytes:append(values: Bytearray | table<int>)

-- Вставляет данные по индексу.
bytes:insert(index: int, value: int)

-- Удаляет элемент(ы), если передан `count`, удалит `count` кол-во элементов с выбранного индекса.
bytes:remove(index: int, [опционально] count: int)

-- Очищает массив
bytes:clear()

-- Создаёт новый Bytearray, содержащий копию части данных с offset до offset+length
bytes:slice(offset: int, length: int) -> Bytearray
```

## View

Это "вьюшки" поверх Bytearray, которые интерпретируют его байты как массив чисел другого размера - **без копирования данных**.

Не имеет своих методов, является просто объектной ссылкой на оригинальный Bytearray.

| Класс        | Тип элементов      | Размер  |
| ------------ | ------------------ | ------- |
| `I16view` | `int16_t` | 2 байта |
| `U16view` | `uint16_t`         | 2 байта |
| `I32view` | `int32_t`          | 4 байта |
| `U32view` | `uint32_t`         | 4 байта |

---

### Пример использования

```lua
local bytes = Bytearray({1, 0, 2, 0, 250, 255})

local u16 = U16view(bytes)
local i16 = I16view(bytes)

print(u16[1]) -- 1
print(u16[2]) -- 2
print(i16[3]) -- -6 (signed вьюшка)

for _, num in ipairs(i16) do
    print(num)
end -- 1; 2; -6
```