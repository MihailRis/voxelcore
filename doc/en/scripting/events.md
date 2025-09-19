# Engine events

## Block events

Callbacks specified in block script.

```lua
function on_placed(x, y, z, playerid)
```

Called on block placed by player

```lua
function on_broken(x, y, z, playerid)
```

Called on block broken by player

```lua
function on_replaced(x, y, z, playerid)
```

Called on block replaced with other by player

```lua
function on_interact(x, y, z, playerid) -> bool
```

Called on block RMB click interaction. Prevents block placing if **true** returned.

```lua
function on_update(x, y, z)
```

Called on block update (near block changed)

```lua
function on_random_update(x, y, z)
```

Called on random block update (grass growth)

```lua
function on_blocks_tick(tps: int)
```

Called tps (20) times per second. Use 1/tps instead of `time.delta()`.

```lua
function on_block_tick(x, y, z, tps: number)
```

Called tps (20 / tick-interval) times per second for a block.
Use 1/tps instead of `time.delta()`.

```lua
function on_player_tick(playerid: int, tps: int)
```

Called tps (20) times per second. Use 1/tps instead of `time.delta()`.

## Item events

Callbacks specified in item script.

```lua
function on_use(playerid: int)
```

Called on RMB click out of a block.

```lua
function on_use_on_block(x: int, y: int, z: int, playerid: int, normal: vec3)
```

Called on block RMB click. Prevents block **placing-block** placing if returns **true**

```lua
function on_block_break_by(x: int, y: int, z: int, playerid: int)
```

Called on block LMB click (unbreakable blocks included).  Prevents block destruction if returns **true**.

## World events

Callbacks specified in *world.lua* script.

```lua
function on_world_open()
```

Called on world open.

```lua
function on_world_save()
```

Called before world save.

```lua
function on_world_tick()
```

Called 20 times per second

```lua
function on_world_quit()
```

Called on world close (after saving)

```lua
function on_block_placed(blockid, x, y, z, playerid)
```

Called on block placed by player

```lua
function on_block_replaced(blockid, x, y, z, playerid)
```

Called on block replaced with other by player

```lua
function on_block_broken(blockid, x, y, z, playerid)
```

Called on block broken by player

```lua
function on_block_interact(blockid, x, y, z, playerid) -> bool
```

Called on block RMB click interaction. Prevents block placing if **true** returned.

### Chunk Events (world.lua)

```lua
function on_chunk_present(x: int, z: int, loaded: bool)
```

Called after a chunk is generated/loaded. If a previously saved chunk is loaded, `loaded` will be true.

```lua
function on_chunk_remove(x: int, z: int)
```

Called when a chunk is unloaded from the world.

### Inventory Events (world.lua)

```lua
function on_inventory_open(invid: int, playerid: int)
```

Called when the inventory is opened. If the inventory was not opened directly by the player, playerid will be -1.

```lua
function on_inventory_closed(invid: int, playerid: int)
```

Called when the inventory is closed.

## Layout events

Script *layouts/layout_name.xml.lua* events.

```lua
function on_open(invid: int, x: int, y: int, z: int)
```

Called on element added to the screen.
invid=0 if no inventory bound
x,y,z=0 if no block bound

```lua
function on_close(invid: int)
```

Called on element removed from the screen.

## HUD events

Callbacks specified in *hud.lua* script.


```lua
function on_hud_open(playerid: int)
```

Called after world open.

```lua
function on_hud_close(playerid: int)
```

Called on world close (before saving)

## *events* library

```lua
events.on(code: str, handler: function)
```

Adds an event handler by its code, not limited to the standard ones.

```lua
events.reset(code: str, [optional] handler: function)
```

Removes the event, adding a handler if specified.

```lua
events.emit(code: str, args...) -> bool
```

Emits an event by code. If the event does not exist, nothing will happen.
The existence of an event is determined by the presence of handlers.

```lua
events.remove_by_prefix(packid: str)
```

Removes all events with the prefix `packid:`. When you exit the world, events from all packs are unloaded, including `core:`.
