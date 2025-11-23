# *item* library

```lua
-- Returns item string ID (name) by index
item.name(itemid: int) -> str

-- Returns item integer ID (index) by name
item.index(name: str) -> int

-- Returns the item display name.
block.caption(blockid: int) -> str

-- Returns the item display description.
item.description(itemid: int) -> str

-- Returns max stack size for the item
item.stack_size(itemid: int) -> int

-- Returns count of available item IDs.
item.defs_count() -> int

-- Returns item icon name to use in 'src' property of an image element
item.icon(itemid: int) -> str

-- Returns the integer id 'placing-block' or 0
item.placing_block(itemid: int) -> int

-- Returns the value of the `model-name` property
item.model_name(itemid: int) -> str

-- Returns item emission property value
item.emission(itemid: int) -> str

-- Returns the value of the `uses` property
item.uses(itemid: int) -> int

-- Checks if an item has specified tag
item.has_tag(itemid: int, tag: str) -> bool
```
