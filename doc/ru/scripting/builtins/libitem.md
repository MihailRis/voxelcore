# Библиотека *item*

```lua
-- Возвращает строковый id предмета по его числовому id (как block.name)
item.name(itemid: int) -> str

-- Возвращает числовой id предмета по строковому id (как block_index)
item.index(name: str) -> int

-- Возвращает название предмета, отображаемое в интерфейсе.
item.caption(itemid: int) -> str

-- Возвращает описание предмета, отображаемое в интерфейсе.
item.description(itemid: int) -> str

-- Возвращает максимальный размер стопки для предмета.
item.stack_size(itemid: int) -> int

-- Возвращает общее число доступных предметов (включая сгенерированные)
item.defs_count() -> int

-- Возвращает имя иконки предмета для использования в свойстве 'src' элемента image
item.icon(itemid: int) -> str

-- Возвращает числовой id блока, назначенного как 'placing-block' или 0
item.placing_block(itemid: int) -> int

-- Возвращает значение свойства `model-name`
item.model_name(itemid: int) -> str

-- Возвращает emission параметр у предмета
item.emission(itemid: int) -> str

-- Возвращает значение свойства `uses`
item.uses(itemid: int) -> int

-- Проверяет наличие тега у предмета
item.has_tag(itemid: int, tag: str) -> bool
```


