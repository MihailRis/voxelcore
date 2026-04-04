# Библиотека *input*


## Основные методы
```lua
-- Возвращает код клавиши по имени, либо -1
input.keycode(keyname: string) -> number

-- Возвращает код кнопки мыши по имени, либо -1
input.mousecode(mousename: string) -> number

-- Проверяет активность ввода по коду, состоящему из:
-- * типа ввода: key (клавиша) или mouse (кнопка мыши)
-- * код ввода: [имя клавиши](#имена-клавиш) или имя кнопки мыши (left, middle, right)
input.is_pressed(code: string) -> boolean

-- Возвращает названия всех доступных привязок.
input.get_bindings() -> table<string>

-- Возвращает текстовое представление кнопки по имени привязки.
input.get_binding_text(bindname: string) -> string

-- Проверяет активность привязки.
input.is_active(bindname: string) -> string

-- Включает/выключает привязку до выхода из мира.
input.set_enabled(bindname: string, flag: boolean)
```

## Курсор мыши
```lua
-- Возвращает позицию курсора на экране.
input.get_mouse_pos() -> {number, number}

-- Возращает дельту позиции курсора.
input.get_mouse_delta() -> {number, number}
```

## Создание обработчиков
```lua
input.add_callback(
    -- Имя привязки
    bindname: string,
    -- Обработчик
    callback: function
    -- UI элемент-владелец обработчика (отвечает за срок жизни)
    [опционально] owner: Element,
    -- Игнорировать захват ввода UI элементами
    [опционально] istoplevel: boolean    
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