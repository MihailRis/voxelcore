# *input* library

```lua
input.keycode(keyname: str) --> int
```

Returns key code or -1 if unknown

```lua
input.mousecode(mousename: str) --> int
```

Returns mouse button code or -1 if unknown

```lua
input.add_callback(
    -- Binding name
    bindname: str,
    -- Handler
    callback: function
    -- UI element that owns the handler (responsible for the handler's lifetime)
    [optional] owner: Element,
    -- Ignore input capture by UI elements
    [optional] istoplevel: bool
)
```

Add binding activation callback. Example:

```lua
input.add_callback("hud.inventory", function ()
	print("Inventory open key pressed")
end)
```

Callback may be added to a key.

```lua
input.add_callback("key:space", function ()
    print("Space pressed")
end)
```

You can also bind the function lifetime to the UI container instead of the HUD. 
In that case, `input.add_callback` may be used until the `on_hud_open` is called.

```lua
input.add_callback("key:escape", function ()
    print("NO")
    return true -- prevents previously assigned functions from being called
end, document.root)
```

```lua
input.get_mouse_pos() --> {int, int}
```

Returns cursor screen position.

```lua
input.get_mouse_delta() --> {int, int}
```

Returns cursor movement delta.

```lua
input.get_bindings() --> strings array
```

Returns all binding names.

```lua
input.get_binding_text(bindname: str) --> str
```

Returns text representation of button by binding name.

```lua
input.is_active(bindname: str) --> bool
```

Checks if the binding is active.

```lua
input.set_enabled(bindname: str, flag: bool)
```

Enables/disables binding until leaving the world.

```lua
input.is_pressed(code: str) --> bool
```

Checks input activity using a code consisting of:
- input type: *key* or *mouse*
- input code: [key name](#key names) or mouse button name (left, middle, right)

Example:
```lua
if input.is_pressed("key:enter") then
    ...
end
```

