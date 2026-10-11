# User input

User input is performed with *bindings* defined with `config/bindings.toml`.

Example:

```toml
packid.binding.name="inputtype:codename"
```

- packid - optional (recommended)
- inputtype - key or mouse
- codename - key or mouse button code (left/right/middle)

## Key names

- space, backspace, tab, enter, caps-lock, escape
- left-ctrl, left-shift, left-alt, left-super
- right-ctrl, right-shift, right-alt, right-super
- delete, home, end, insert, page-up, page-down
- left, right, down, up
- a..z
- 0..9
- f1..f25
- grave-accent, minus, equal, left-bracket, right-bracket, backslash
- semicolon, apostrophe, comma, period, slash

## *input* library

See [*input* library](builtins/libinput.md)
