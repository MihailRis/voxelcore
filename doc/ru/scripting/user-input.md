# Пользовательский ввод

Обработка нажатий клавиш и кнопок мыши обрабатываются через привязки (bindings), которые назначаются в паке, в файле `config/bindings.toml` в формате:

```toml
packid.binding.name="inputtype:codename"
```

- packid - опционально, но желательно
- inputtype - key или mouse
- codename - имя клавиши или кнопки мыши (left/right/middle)

## Имена клавиш

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

## Библиотека input

См. [библиотека *input*](builtins/libinput.md)
