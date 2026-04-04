# *rules* library

```lua
rules.create(
    -- rule name
    name: str,
    -- default value
    default: bool,
    -- value change handler function
    [optional] handler: function
) -> int
```

Creates a rule. If a handler is specified, returns the id for deletion.

> [!NOTE]
> A rule is created by calling rules.create with a default value.
> Rules that have not been created can be used, but resetting via rules.reset will result in setting the value to nil.

```lua
rules.listen(
    -- rule name
    name: str,
    -- value change handler function
    handler: function
) -> int
```

Adds a rule value change handler.

Returns the id for deletion.
Also allows subscribing to a rule before it is created.

```lua
rules.unlisten(name: str, id: int)
```

Removes a rule handler by id, if it exists.

```lua
rules.get(name: str) -> bool | nil
```

Returns the rule value, or nil if it has not been created yet.

```lua
rules.set(name: str, value: bool)
```

Sets the rule value by calling handlers. Can be used before
creating a rule.

```lua
rules.reset(name: str)
```

Resets the rule value to the default value.

## Standard Rules

| Name                   | Description                                                 | Default |
| ---------------------- | ----------------------------------------------------------- | ------- |
| allow-cheats           | Allow commands whose names are in the console.cheats array. | true    |
| allow-content-access   | Allow the content access panel.                             | true    |
| allow-flight           | Allow flight.                                               | true    |
| allow-noclip           | Allow noclip.                                               | true    |
| allow-attack           | Allow attacking entities.                                   | true    |
| allow-destroy          | Allow block destruction.                                    | true    |
| allow-cheat-movement   | Allow special quick movement keys.                          | true    |
| allow-debug-cheats     | Allow cheat controls in the debug panel.                    | true    |
| allow-fast-interaction | Allow fast interaction.                                     | true    |
