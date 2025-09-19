# События движка

## События блоков

Функции для обработки событий, прописываемые в скрипте блока.

```lua
function on_placed(x, y, z, playerid)
```

Вызывается после установки блока игроком

```lua
function on_broken(x, y, z, playerid)
```

Вызывается после разрушения блока игроком

```lua
function on_replaced(x, y, z, playerid)
```

Вызывается после замены блока игроком

```lua
function on_interact(x, y, z, playerid) -> bool
```

Вызывается при нажатии на блок ПКМ. Предотвращает установку блоков, если возвращает `true`

```lua
function on_update(x, y, z)
```

Вызывается при обновлении блока (если изменился соседний блок)

```lua
function on_random_update(x, y, z)
```

Вызывается в случайные моменты времени (рост травы на блоках земли)  

```lua
function on_blocks_tick(tps: int)
```

Вызывается tps (20) раз в секунду. Используйте 1/tps вместо `time.delta()`.

```lua
function on_block_tick(x, y, z, tps: number)
```

Вызывается tps (20 / tick-interval) раз в секунду для конкретного блока.
Используйте 1/tps вместо `time.delta()`.

```lua
function on_player_tick(playerid: int, tps: int)
```

Вызывается tps (20) раз в секунду. Используйте 1/tps вместо `time.delta()`.

## События предметов

Функции для обработки событий, прописываемые в скрипте предмета.

```lua
function on_use(playerid: int)
```

Вызывается при нажатии ПКМ не на блок.

```lua
function on_use_on_block(x: int, y: int, z: int, playerid: int, normal: vec3)
```

Вызывается при нажатии ПКМ на блок. Предотвращает установку блока, прописанного в `placing-block` если возвращает `true`

```lua
function on_block_break_by(x: int, y: int, z: int, playerid: int)
```

Вызывается при нажатии ЛКМ на блок (в т.ч неразрушимый). Предотвращает разрушение блока, если возвращает `true`

## События мира  

События мира для контент-пака прописываются в `scripts/world.lua`

```lua
function on_world_open()
```

Вызывается при загрузке мира

```lua
function on_world_save()
```  

Вызывается перед сохранением мира

```lua
function on_world_tick()
```

Вызывается 20 раз в секунду

```lua
function on_world_quit()
```

Вызывается при выходе из мира (после сохранения)

```lua
function on_block_placed(blockid, x, y, z, playerid)
```

Вызывается после установки блока игроком

```lua
function on_block_replaced(blockid, x, y, z, playerid)
```

Вызывается после замены блока игроком

```lua
function on_block_broken(blockid, x, y, z, playerid)
```

Вызывается после разрушения блока игроком

```lua
function on_block_interact(blockid, x, y, z, playerid) -> bool
```

Вызывается при нажатии на блок ПКМ. Предотвращает установку блоков, если возвращает `true`

### События чанков (world.lua)

```lua
function on_chunk_present(x: int, z: int, loaded: bool)
```

Вызывается после генерации/загрузки чанка. В случае загрузки ранее сохраненного чанка `loaded` будет истинным.

```lua
function on_chunk_remove(x: int, z: int)
```

Вызывается при выгрузке чанка из мира.

### События инвентарей (world.lua)

```lua
function on_inventory_open(invid: int, playerid: int)
```

Вызывается при открытии инвентаря. Если инвентарь был открыт не напрямую игроком, playerid будет равен -1.

```lua
function on_inventory_closed(invid: int, playerid: int)
```

Вызывается при закрытии инвентаря.


## События макета

События прописываются в файле `layouts/имя_макета.xml.lua`.

```lua
function on_open(invid: int, x: int, y: int, z: int)
```

Вызывается при добавлении элемента на экран.
- При отсутствии привязки к инвентарю invid будет равен 0.
- При отсутствии привязки к блоку x, y, z так же будут равны 0. 

```lua
function on_close(invid: int)
```

Вызывается при удалении элемента с экрана.

## События HUD  

События связанные с игровым интерфейсом прописываются в файле `scripts/hud.lua`

```lua
function on_hud_open(playerid: int)
```

Вызывается после входа в мир, когда становится доступна библиотека hud. Здесь на экран добавляются постоянные элементы.

```lua
function on_hud_close(playerid: int)
```

Вызывается при выходе из мира, перед его сохранением.

## Библиотека *events*

```lua
events.on(code: str, handler: function)
```

Добавляет обработчик события по его коду, не ограничиваясь стандартными.

```lua
events.reset(code: str, [опционально] handler: function)
```

Удаляет событие, добавляя обработчик, если указан.

```lua
events.emit(code: str, args...) -> bool
```

Генерирует событие по коду. Если событие не существует, ничего не произойдет.
Существование события определяется наличием обработчиков.

```lua
events.remove_by_prefix(packid: str)
```

Удаляет все события с префиксом `packid:`. Вы выходе из мира выгружаются события всех паков, включая `core:`.
