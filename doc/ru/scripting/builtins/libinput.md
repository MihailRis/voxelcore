# Библиотека *input*

```lua
input.keycode(keyname: str) --> int
```

Возвращает код клавиши по имени, либо -1

```lua
input.mousecode(mousename: str) --> int
```

Возвращает код кнопки мыши по имени, либо -1

```lua
input.add_callback(
    -- Имя привязки
    bindname: str,
    -- Обработчик
    callback: function
    -- UI элемент-владелец обработчика (отвечает за срок жизни)
    [опционально] owner: Element,
    -- Игнорировать захват ввода UI элементами
    [опционально] istoplevel: bool    
)
```

Назначает функцию, которая будет вызываться при активации привязки. Пример:

```lua
input.add_callback("hud.inventory", function ()
    print("Inventory open key pressed")
end)
```

Можно назначить функцию на нажатие клавиши.

```lua
input.add_callback("key:space", function ()
    print("Space pressed")
end)
```

Также можно привязать время жизни функции к UI контейнеру, вместо HUD.
В таком случае, `input.add_callback` можно использовать до вызова `on_hud_open`.

```lua
input.add_callback("key:escape", function ()
    print("NO")
    return true -- предотвращает вызов назначенных ранее функций
end, document.root)
```

```lua
input.get_mouse_pos() --> {int, int}
```

Возвращает позицию курсора на экране.

```lua
input.get_mouse_delta() --> {int, int}
```

Возращает дельту позиции курсора.

```lua
input.get_bindings() --> массив строк
```

Возвращает названия всех доступных привязок.

```lua
input.get_binding_text(bindname: str) --> str
```

Возвращает текстовое представление кнопки по имени привязки.

```lua
input.is_active(bindname: str) --> bool
```

Проверяет активность привязки.

```lua
input.set_enabled(bindname: str, flag: bool)
```

Включает/выключает привязку до выхода из мира.

```lua
input.is_pressed(code: str) --> bool
```

Проверяет активность ввода по коду, состоящему из:
- типа ввода: key (клавиша) или mouse (кнопка мыши)
- код ввода: [имя клавиши](#имена-клавиш) или имя кнопки мыши (left, middle, right)

Пример: 
```lua
if input.is_pressed("key:enter") then
    ...
end
```
