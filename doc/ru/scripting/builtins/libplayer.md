## Содержание
- [Управление игроками](#управление-игроками)
- [Позиция и движение](#позиция-и-движение)
- [Режимы и свойства](#режимы-и-свойства)
- [Инвентарь и взаимодействие](#инвентарь-и-взаимодействие)
- [Камеры](#камеры)
- [Поиск игроков](#поиск-игроков)

## Управление игроками

```lua
-- Создает игрока и возвращает его ID
player.create(name: string) -> int

-- Удаляет игрока по ID
player.delete(id: int)

-- Возвращает ID сущности игрока
player.get_entity(playerid: int) -> int


-- Сеттер и геттер точки спавна игрока
player.set_spawnpoint(playerid: int, x: number, y: number, z: number)
player.get_spawnpoint(playerid: int) -> number, number, number


-- Сеттер и геттер имени игрока
player.set_name(playerid: int, name: string)
player.get_name(playerid: int) -> string


-- Сеттер и геттер статуса "заморозки" игрока.
-- При заморозке сущность игрока удаляется. При последующей разморозке будет создана новая, с другим id.
player.set_suspended(pid: int, suspended: boolean)
player.is_suspended(pid: int) -> boolean
```

## Позиция и движение

```lua
-- Сеттер и геттер позиции игрока
player.set_pos(playerid: int, x: number, y: number, z: number)
player.get_pos(playerid: int) -> number, number, number


-- Сеттер и геттер линейной скорости игрока
player.set_vel(playerid: int, x: number, y: number, z: number)
player.get_vel(playerid: int) -> number, number, number

-- Сеттер и геттер вращения камеры игрока
player.set_rot(playerid: int, x: number, y: number, z: number)
player.get_rot(playerid: int, interpolated: boolean) -> number, number, number


-- Возвращает вектор направления взгляда игрока
player.get_dir(playerid: int) -> vec3
```

## Режимы и свойства

```lua
-- Сеттер и геттер режима полета
player.set_flight(playerid: int, boolean)
player.is_flight(playerid: int) -> boolean


-- Сеттер и геттер режима noclip
player.set_noclip(playerid: int, boolean)
player.is_noclip(playerid: int) -> boolean


-- Сеттер и геттер бесконечных предметов (не удаляются из инвентаря при использовании)
player.set_infinite_items(playerid: int, boolean)
player.is_infinite_items(playerid: int) -> boolean


-- Сеттер и геттер мнгновенного разрушения блоков при активации привязки player.destroy.
player.set_instant_destruction(playerid: int, boolean)
player.is_instant_destruction(playerid: int) -> boolean


-- Сеттер и геттер свойства, определяющего, прогружает ли игрок чанки вокруг себя.
player.set_loading_chunks(playerid: int, boolean)
player.is_loading_chunks(playerid: int) -> boolean


-- Сеттер и геттер свойства, определяющего максимальную дистанцию взаимодействия.
player.set_interaction_distance(playerid: int, distance: number)
player.get_interaction_distance(playerid: int) -> number
```

## Инвентарь и взаимодействие

```lua
-- Возвращает ID инвентаря и индекс выбранного слота
player.get_inventory(playerid: int) -> int, int

-- Устанавливает выбранный слот
player.set_selected_slot(playerid: int, slotid: int)

-- Возвращает позицию выбранного блока
player.get_selected_block(playerid: int) -> x,y,z

-- Возвращает ID выбранной сущности
player.get_selected_entity(playerid: int) -> int
```

## Камеры

```lua
-- Возвращает индекс текущей камеры игрока
player.get_camera(playerid: int) -> int

-- Переключает камеры игрока
player.set_camera(playerid: int, camera_index: int)
```

## Поиск игроков

```lua
-- Возвращает массив id игроков в пределах сферы с центром center и радиусом radius.
player.get_all_in_radius(center: vec3, radius: number) -> table<int>

-- Возвращает массив id всех активных игроков.
player.get_all() -> table<int>

-- Возвращает id ближайшего к указанной позиции игрока, либо nil если игроков нет.
player.get_nearest(position: vec3) -> int / nil
```
