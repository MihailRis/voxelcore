# Библиотека *item*

```lua
-- Возвращает строковый id предмета по его числовому id (как block.name)
item.name(itemid: int) -> string

-- Возвращает числовой id предмета по строковому id (как block_index)
item.index(name: string) -> int

-- Возвращает название предмета, отображаемое в интерфейсе.
item.caption(itemid: int) -> string

-- Возвращает описание предмета, отображаемое в интерфейсе.
item.description(itemid: int) -> string

-- Возвращает максимальный размер стопки для предмета.
item.stack_size(itemid: int) -> int

-- Возвращает общее число доступных предметов (включая сгенерированные)
item.defs_count() -> int

-- Возвращает имя иконки предмета для использования в свойстве 'src' элемента image
item.icon(itemid: int) -> string

-- Возвращает числовой id блока, назначенного как 'placing-block' или 0
item.placing_block(itemid: int) -> int

-- Возвращает значение свойства `model-name`
item.model_name(itemid: int) -> string

-- Возвращает emission параметр у предмета
item.emission(itemid: int) -> string

-- Возвращает значение свойства `uses`
item.uses(itemid: int) -> int

-- Проверяет наличие тега у предмета
item.has_tag(itemid: int, tag: string) -> boolean
```