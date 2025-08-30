# Библиотека gfx.posteffects

Библиотека для работы с эффектами пост-обработки.

Слот эффектов является ресурсом, и должен быть объявлен в resources.json в корневой директории пака:

```json
{
    "post-effect-slot": [
        "имя_слота"
    ]
}
```

```lua
-- Возвращает индекс слота эффектов по имени (пак:имя_слота).
-- При отсутствии указанного слота возвращает -1
gfx.posteffects.index(name: str) --> int

-- Назначает эффект на слот
gfx.posteffects.set_effect(slot: int, effect: str)

-- Возвращает интенсивность эффекта (от 0.0 до 1.0)
-- Если слот пуст, возвращает 0.0
gfx.posteffects.get_intensity(slot: int) --> number

-- Устанавливает интенсивность эффекта (от 0.0 до 1.0)
-- (Корректность обработки параметра между значениями 0.0 и 1.0 зависит
-- от эффекта
gfx.posteffects.set_intensity(slot: int, intensity: number)

-- Возвращает true если слот не пуст и интенсивность эффекта ненулевая
gfx.posteffects.is_active(slot: int) --> bool

-- Устанавливает значения параметров (директивы 'param')
gfx.posteffects.set_params(slot: int, params: table)

-- Устанавливает значения в массив
gfx.posteffects.set_array(
    -- индекс слота эффектов
    slot: int,
    -- имя параметра (массива)
    name: str
    -- строка данных (используйте функцию Bytearray_as_string)
    data: str
)

-- Пример заполнения массива `#param vec3 u_имяМассива[64]`:
local buffer = Bytearray(0)
for i = 0, 63 do
    local x = math.random() * 2.0 - 1.0
    local y = math.random() * 2.0 - 1.0
    local z = math.random() * 2.0 - 1.0
    Bytearray.append(buffer, byteutil.pack("fff", x, y, z))
end
gfx.posteffects.set_array(slot, "u_имяМассива", Bytearray_as_string(buffer))
```
