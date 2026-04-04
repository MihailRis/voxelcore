# Библиотека *block*

## Содержание
- [Таблицы и прочие общие методы](#таблицы-и-прочие-общие-методы)
- [Работа с миром](#работа-с-миром)
- [Свойства блоков](#свойства-блоков)
- [Raycast](#raycast)
- [Вращение](#вращение)
- [Расширенные блоки](#расширенные-блоки)
- [Модель и физика](#модель-и-физика)
- [Данные блоков](#данные-блоков)

## Таблицы и прочие общие методы
```lua
-- Возвращает количество id доступных в загруженном контенте блоков
block.defs_count() -> int

-- Таблица материалов по их полным именам (пример: base:carpet)
block.materials: table<string, table>

-- Таблица пользовательских свойств блоков (см. ../../block-properties.md)
block.properties: table<int, table<string, any>>
```


## Работа с миром
```lua
-- Возвращает числовой id блока на указанных координатах.
-- Если чанк на указанных координатах не загружен, возвращает -1.
block.get(x: int, y: int, z: int) -> int

-- Устанавливает блок с заданным числовым id и состоянием (0 - по-умолчанию) на заданных координатах.
-- Если передан noupdate=true, то вызов ивента `on_update` для соседних блоков не произойдёт.
block.set(x: int, y: int, z: int, id: int, states: int, noupdate: boolean=false)

-- Устанавливает блок с заданным числовым id и состоянием (0 - по-умолчанию) на заданных координатах
-- от лица игрока, вызывая событие on_placed.
-- playerid не является обязательным
block.place(x: int, y: int, z: int, id: int, states: int, [опционально] playerid: int)

-- Ломает блок на заданных координатах от лица игрока, вызывая событие on_broken.
-- playerid не является обязательным
block.destruct(x: int, y: int, z: int, playerid: int)

-- Возвращает индекс варианта блока
block.get_variant(x: int, y: int, z: int) -> int

-- Устанавливает вариант блока по индексу
block.set_variant(x: int, y: int, z: int, index: int)
```

> [!WARNING]
> `block.set` не вызывает событие on_placed.

## Свойства блоков
```lua
-- Проверяет, является ли блок на указанных координатах полным
block.is_solid_at(x: int, y: int, z: int) -> boolean

-- Проверяет, можно ли на заданных координатах поставить блок 
-- (примеры: воздух, трава, цветы, вода)
block.is_replaceable_at(x: int, y: int, z: int) -> boolean

-- Возвращает полное состояние (поворот + сегмент + доп. информация) в виде целого числа
block.get_states(x: int, y: int, z: int) -> int

-- Возвращает название блока, отображаемое в интерфейсе.
block.caption(blockid: int) -> string

-- Возвращает строковый id блока по его числовому id.
block.name(blockid: int) -> string

-- Возвращает числовой id блока, принимая в качестве агрумента строковый
block.index(name: string) -> int

-- Собирает полное состояние в виде целого числа
block.compose_state(state: {rotation: int, segment: int, userbits: int}) -> int

-- Разбирает полное состояние на: вращение, сегмент, пользовательские биты
block.decompose_state(state: int) -> {int, int, int}

-- Проверяет наличие тега у блока
block.has_tag(id: int, tag: string) -> boolean

-- Возвращает числовой id предмета, указанного в свойстве *picking-item*.
block.get_picking_item(id: int) -> int

-- Возвращает id материала блока.
block.material(blockid: int) -> string
```

## Raycast

```lua
block.raycast(start: vec3, dir: vec3, max_distance: number, [опционально] dest: table, 
    [опционально] filter: table, [опционально] include_non_selectable = false
) -> {
    block: int, -- id блока
    endpoint: vec3, -- точка касания луча
    iendpoint: vec3, -- позиция блока, которого касается луч
    length: number, -- длина луча
    normal: vec3, -- вектор нормали поверхности, которой касается луч
} или nil
```

Бросает луч из точки start в направлении dir. Max_distance указывает максимальную длину луча.

Аргумент `filter` позволяет указать какие блоки являются "прозрачными" для луча, прим.: {"base:glass","base:water"}. 
Для использования агрумент `dest` нужно чем-то заполнить(можно nil), это сделано для обратной совместимости

Аргумент `include_non_selectable` определяет, будут ли учтены блоки, которые нельзя выбрать курсором.
Пример - `base:water`

Функция возвращает таблицу с результатами или nil, если луч не касается блока.

Для результата будет использоваться целевая (dest) таблица вместо создания новой, если указан опциональный аргумент.

## Вращение

```lua
-- Возвращает целочисленный единичный вектор X блока на указанных координатах с учётом его вращения (три целых числа).
-- Если поворот отсутствует, возвращает 1, 0, 0
block.get_X(x: int, y: int, z: int) -> int, int, int
block.get_X(id: int, rotation: int) -> int, int, int

-- То же, но для оси Y (по-умолчанию 0, 1, 0)
block.get_Y(x: int, y: int, z: int) -> int, int, int
block.get_Y(id: int, rotation: int) -> int, int, int

-- То же, но для оси Z (по-умолчанию 0, 0, 1)
block.get_Z(x: int, y: int, z: int) -> int, int, int
block.get_Z(id: int, rotation: int) -> int, int, int

-- Возвращает индекс поворота блока в его профиле вращения (не превышает 7).
block.get_rotation(x: int, y: int, z: int) -> int

-- Устанавливает вращение блока по индексу в его профиле вращения.
block.set_rotation(x: int, y: int, z: int, rotation: int)

-- Возвращает имя профиля вращения (none/pane/pipe)
block.get_rotation_profile(id: int) -> string
```

## Расширенные блоки

```lua
-- Проверяет, является ли блок расширенным.
block.is_extended(id: int) -> boolean

-- Возвращает размер блока.
block.get_size(id: int) -> int, int, int

-- Проверяет является ли блок сегментом расширенного блока, не являющимся главным.
block.is_segment(x: int, y: int, z: int) -> boolean

-- Возвращает позицию главного сегмента расширенного блока или исходную позицию,
-- если блок не является расширенным.
block.seek_origin(x: int, y: int, z: int) -> int, int, int
```

## Модель и физика

```lua
-- возвращает тип модели блока (block/aabb/custom/...)
block.get_model(id: int, [опционально] variant_index: int = 0) -> string

-- возвращает имя модели блока
block.model_name(id: int, [опционально] variant_index: int = 0) -> string

-- возвращает массив из 6 текстур, назначенных на стороны блока
block.get_textures(id: int, [опционально] variant_index: int = 0) -> table<string>

-- Возвращает массив из двух векторов (массивов из 3 чисел):
-- 1. Минимальная точка хитбокса
-- 2. Размер хитбокса
-- rotation_index - индекс поворота блока
-- hitbox_index - индекс хитбокса
block.get_hitbox(id: int, rotation_index: int, [опционально] hitbox_index: int = 0) -> {vec3, vec3}
```

## Данные блоков

```lua
-- Возвращает выбранное число бит с указанного смещения в виде целого беззнакового числа
block.get_user_bits(x: int, y: int, z: int, offset: int, bits: int) -> int

-- Записывает указанное число бит значения value в user bits по выбранному смещению
block.set_user_bits(x: int, y: int, z: int, offset: int, bits: int, value: int) -> int


-- записывает значение в указанное поле блока
-- * бросает исключение при несовместимости типов
-- * бросает исключение при выходе за границы массива
-- * ничего не делает при отсутствии поля у блока
block.set_field(
    x: int, y: int, z: int, 
    name: string,
    value: string | number | boolean, 
    [опционально] index: int = 0
)

-- возвращает значение записанное в поле блока
-- * возвращает nil если:
--     1. поле не существует
--     2. ни в одно поле блока не было произведено записи
-- * бросает исключение при выходе за границы массива
block.get_field(
    x: int, y: int, z: int, 
    name: string, 
    [опционально] index: int = 0
) -> any | nil
```