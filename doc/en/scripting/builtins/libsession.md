# *session* Library

This library is used to store data that is not reset when loading/unloading content, and is a recommended alternative to using the `_G` table.

The stored data is not saved when the engine terminates.

To minimize the risk of name collisions, it is recommended to use pack prefixes (`pack_id:table_name`), or, if a pack stores a single table in *session*, use the pack name as the entry name.

```lua
-- Returns a table by name. Creates an empty table if it does not exist.
session.get(name: str) -> table

-- Deletes a table by name.
session.reset(name: str)

-- Returns true if the table has already been created by calling session.get
session.has(name: str) -> boolean
```
