# UI properties and methods

UI elements in scripts are accessed through a Document instance
(*document* variable) by id specified in xml.

Example: print the pos property of an element with id: "worlds-panel" to the console:
```lua
print(document["worlds-panel"].pos)
-- or
local worldsPanel = document["worlds-panel"]
print(worldsPanel.pos)
```

The IDs of the elements are global for the document, that is, *worlds-panel* can be located either in the root element,
and in a nested container.

The element id cannot be changed from a script.

The following tables will use abbreviated type descriptions, such as:
- vec2 - an array of two numbers.
- ivec2 - an array of two integers.
- rgba - an array of four integers in the range `[0..255]` denoting RGBA constituent colors.

Element methods, according to OOP features in Lua, are called using the `:` operator instead of `.`

For example:
```lua
document["worlds-panel"]:clear()
```

Access to nested elements is performed by index (starting from one).

## General properties and methods

Properties that apply to all elements:

| Name          | Type    | Read | Write | Description                                 |
| ------------- | ------- | ---- | ----- | ------------------------------------------- |
| id            | string  | yes  | *no*  | element id                                  |
| pos           | vec2    | yes  | yes   | element position inside a container         |
| wpos          | vec2    | yes  | yes   | element position inside the window          |
| size          | vec2    | yes  | yes   | element size                                |
| interactive   | bool    | yes  | yes   | ability to interact with the element        |
| enabled       | bool    | yes  | yes   | visually indicated version of *interactive* |
| visible       | bool    | yes  | yes   | element visibility                          |
| focused       | bool    | yes  | yes   | focus on element                            |
| color         | rgba    | yes  | yes   | element color                               |
| hoverColor    | rgba    | yes  | yes   | hover color                                 |
| pressedColor  | rgba    | yes  | yes   | color when pressed                          |
| tooltip       | string  | yes  | yes   | tooltip text                                |
| tooltipDelay  | float   | yes  | yes   | tooltip delay                               |
| contentOffset | vec2    | yes  | *no*  | element content offset                      |
| cursor        | string  | yes  | yes   | cursor displayed on hover                   |
| parent        | Element | yes  | *no*  | parent element or nil                       |

Common element methods:

| Method              | Description                                                                         |
| ------------------- | ----------------------------------------------------------------------------------- |
| moveInto(container) | moves the element to the specified container (the element is specified, not the id) |
| destruct()          | removes element                                                                     |
| reposition()        | updates the element position based on the `positionfunc`                            |

## Containers

Common properties for containers (elements: container, panel, button, pagebox):

| Name   | Type   | Read | Write | Description     |
| ------ | ------ | ---- | ----- | --------------- |
| scroll | string | yes  | yes   | scroll contents |

Common methods:

| Method                          | Description                                                                                  |
| ------------------------------- | -------------------------------------------------------------------------------------------- |
| clear()                         | clears content                                                                               |
| add(xml)                        | adds an element, creating it using xml code. Example: `container:add("<image src='test'/>")` |
| add(xml, data)                  | overload with table, which in events declared in xml will be available as DATA               |
| setInterval(interval, callback) | assigns a function to be executed repeatedly at an interval specified in milliseconds        |

## Textbox

Properties:

| Name        | Type   | Read | Write | Description                                                                          |
| ----------- | ------ | ---- | ----- | ------------------------------------------------------------------------------------ |
| text        | string | yes  | yes   | entered text or placeholder                                                          |
| placeholder | string | yes  | yes   | placeholder (used if nothing has been entered)                                       |
| hint        | string | yes  | yes   | text to display when nothing is entered                                              |
| caret       | int    | yes  | yes   | carriage position. `textbox.caret = -1` will set the position to the end of the text |
| editable    | bool   | yes  | yes   | text mutability                                                                      |
| edited      | bool   | yes  | yes\* | is text edited since the last set / edited status reset                              |
| multiline   | bool   | yes  | yes   | multiline support                                                                    |
| lineNumbers | bool   | yes  | yes   | display line numbers                                                                 |
| textWrap    | bool   | yes  | yes   | automatic text wrapping (only with multiline: "true")                                |
| valid       | bool   | yes  | no    | is the entered text correct                                                          |
| textColor   | vec4   | yes  | yes   | text color                                                                           |
| syntax      | string | yes  | yes   | syntax highlighting ("lua" - Lua)                                                    |
| markup      | string | yes  | yes   | text markup language ("md" - Markdown)                                               |

\* - false only

Methods:

| Method                    | Description                                                      |
| ------------------------- | ---------------------------------------------------------------- |
| paste(text: str)          | inserts the specified text at the caret position                 |
| lineAt(pos: int) -> int   | determines the line number by position in the text               |
| linePos(line: int) -> int | determines the position of the beginning of the line in the text |

## Slider (trackbar)

Properties:

| Name       | Type  | Read | Write | Description           |
| ---------- | ----- | ---- | ----- | --------------------- |
| value      | float | yes  | yes   | current value         |
| min        | float | yes  | yes   | minimum value         |
| max        | float | yes  | yes   | maximum value         |
| step       | float | yes  | yes   | division step         |
| trackWidth | float | yes  | yes   | control element width |
| trackColor | rgba  | yes  | yes   | control element color |

## Menu (pagebox)

Properties:

| Name  | Type   | Read | Write | Description  |
| ----- | ------ | ---- | ----- | ------------ |
| page  | string | yes  | yes   | current page |

Methods:

| Method  | Description                       |
| ------- | --------------------------------- |
| back()  | switches to previous page         |
| reset() | resets page and switching history |

## Checkbox

Properties:

| Name    | Type | Read | Write | Description |
| ------- | ---- | ---- | ----- | ----------- |
| checked | bool | yes  | yes   | mark status |

## Button

Properties:

| Name  | Type   | Read | Write | Description  |
| ----- | ------ | ---- | ----- | ------------ |
| text  | string | yes  | yes   | button text  |

## Label

Properties:

| Name   | Type   | Read | Write | Description                            |
| ------ | ------ | ---- | ----- | -------------------------------------- |
| text   | string | yes  | yes   | label text                             |
| markup | string | yes  | yes   | text markup language ("md" - Markdown) |

## Image

Properties:

| Name   | Type   | Read | Write | Description      |
| ------ | ------ | ---- | ----- | ---------------- |
| src    | string | yes  | yes   | texture name     |
| region | vec4   | yes  | yes   | image sub-region |

## Canvas

Properties:

| Title | Type   | Read | Write | Description |
|-------|--------| ---- |-------|-------------|
| data  | Canvas | yes  | no    | canvas data |

Methods:

Here, *color* can be specified in the following ways:
- rgba: int
- r: int, g: int, b: int
- r: int, g: int, b: int, a: int

| Method                                                   | Description                                             |
|----------------------------------------------------------|---------------------------------------------------------|
| data:at(x: int, y: int)                                  | returns an RGBA pixel at the given coordinates          |
| data:set(x: int, y: int, *color*)                        | updates an RGBA pixel at the given coordinates          |
| data:line(x1: int, y1: int, x2: int, y2: int, *color*)   | draws a line with the specified RGBA color              |
| data:blit(src: Canvas, dst_x: int, dst_y: int)           | draws the src canvas at the specified coordinates       |
| data:clear()                                             | clears the canvas                                       |
| data:clear(*color*)                                      | fills the canvas with the specified RGBA color          |
| data:update()                                            | applies changes to the canvas and uploads it to the GPU |
| data:set_data(data: table<int>)                          | replaces pixel data (width * height * 4 numbers)        |
| data:create_texture(name: str)                           | creates and shares texture to renderer                  |

## Inline frame (iframe)

| Name     | Type   | Read | Write | Description                 |
|----------|--------|------|-------|-----------------------------|
| src      | string | yes  | yes   | id of the embedded document |

## Select

Derived from button with access to properties such as the text to display.

Properties:

| Name    | Type   | Read | Write | Description                                      |
|---------|--------|------|-------|--------------------------------------------------|
| value   | string | yes  | yes   | Selected value                                   |
| options | table  | yes  | yes   | List of options (tables `{value=..., text=...}`) |

## Inventory

Properties:

| Name      | Type | Read | Write | Description                                       |
| --------- | ---- | ---- | ----- | ------------------------------------------------- |
| inventory | int  | yes  | yes   | id of the inventory to which the element is bound |
