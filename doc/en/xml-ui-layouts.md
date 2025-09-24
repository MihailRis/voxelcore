# XML UI Building

See also [ui elements in scripting](scripting/ui.md).

# Specific types

**2D vector** - pair of numbers separated with comma.
Examples:
- `"500,200"`
- `"0.4,53.01"`
- `"0,0"`

**3D vector** - three numbers separated with comma.
Examples:
- `"60,30,53"`
- `"0.4,0.1,0.753"`

**4D vector** - four numbers separated with comma.
Examples:
- `"10,5,10,3"`
- `"0.1,0.5,0.0,0.0"`

**RGBA color** - only HEX notation available
Examples:
- `"#FF8000"` - opaque orange
- `"#FFFFFF80"` - semi-transparent white
- `"#000000FF"` - opaque black

# Common element attributes

- `id` - element identifier. Type: string.
- `pos` - element position. Type: 2D vector.
- `size` - element size. Type: 2D vector.
- `color` - element color. Type: RGBA color.
- `margin` - element margin. Type: 4D vector
  *left, top, right, bottom*
- `visible` - element visibility. Type: boolean (true/false)
- `min-size` - minimal element size. Type: 2D vector.
- `position-func` - position supplier for an element (two numbers), called on every parent container size update or on element adding on a container. May be called before *on_hud_open*
- `size-func` - element size provider (two numbers), called when the size of the container in which the element is located changes, or when an element is added to the container. Can be called before on_hud_open is called.
- `onclick` - lua function called when an element is clicked.
- `ondoubleclick` - lua function called when you double click on an element.
- `onfocus` - lua function called when focusing on an element.
- `ondefocus` - lua function called when the element loses focus.
- `tooltip` - tooltip text
- `tooltip-delay` - tooltip show-up delay
- `gravity` - automatic positioning of the element in the container. (Does not work in automatic containers like panel). Values: *top-left, top-center, top-right, center-left, center-center, center-right, bottom-left, bottom-center, bottom-right*.
- `z-index` - determines the order of elements, with a larger value it will overlap elements with a smaller one.
- `interactive` - if false, hovering over the element and all sub-elements will be ignored.
- `cursor` - the cursor displayed when hovering over the element (arrow/text/pointer/crosshair/ew-resize/ns-resize/...).

# Template attributes

- `if` with values ​​('', 'false', 'nil') the element will be ignored, including sub-elements.
- `ifnot` is the same as `if`, but with the opposite condition.

# Common *container* attributes

Buttons and panels are also containers.

- `scrollable` - element scrollability. Type: boolean.
- `scroll-step` - scrolling step. Type: integer.

# Common *panel* attributes

Buttons are also panels.

- `padding` - element padding. Type: 4D vector.
  *left, top, right, bottom*
- `max-length` - maximal length of panel stretching before scrolling (if scrollable = true). Type: number.
- `min-length` - minimal length of panel. Type: number.
- `orientation` - panel orientation: horizontal/vertical.
- `interval` - space between elements. Type: number.

# Common elements

## *container*

A primitive container.
Use static positions / `gravity` / script to position elements.

## *panel*

A simple panel. Arranges elements in the order defined by the `orientation` attribute, adjusting the width (vertical) or height (horizontal) to the panel width.

## *splitbox* (splitter)

A panel that controls the size and position of two elements. Allows the user to move the splitter, changing the size of the elements.

- `split-pos` - the initial position of the splitter from 0.0 to 1.0.

## *button*

Inner text is a button text.

- `text-align` - inner text alignment (*left/center/right*). Type: string.

## *checkbox*

- `checked` - defines the checked state.
- `supplier` - mark state supplier (called every frame).
- `consumer` - lua function-consumer of the state of the mark.

## *label*

- `valign` - vertical text alignment: top/center/bottom.
- `supplier` - text supplier (called every frame).
- `autoresize` - automatic change of element size (default - false). Does not affect font size.
- `multiline` - allows display of multiline text.
- `text-wrap` - allows automatic text wrapping (works only with multiline: "true").
- `markup` - text markup language ("md" - Markdown).

## *image*

- `src` - name of an image stored in textures folder. Extension is not specified. Type: string.
  Example: *gui/error*
- `region` - image region x1, y1, x2, y2 from 0.0, 0.0 (upper left corner), 1.0, 1.0 (lower right corner)

## *canvas*

- _No additional attributes_

## *textbox*

Inner text - initially entered text

- `placeholder` - placeholder text (used if the text field is empty)
- `hint` - text displayed if the text field is empty (not sent to consumer, sub-consumer and validator).
- `supplier` - text supplier (called every frame)
- `consumer` - lua function that receives the entered text. Called only when input is complete
- `sub-consumer` - lua function-receiver of the input text. Called during text input or deletion.
- `oncontrolkey` - lua function called for combinations of the form (Ctrl + ?). The codepoint of the second key is given as the first argument.
The key code for comparison can be obtained via `input.keycode("key_name")`
- `autoresize` - automatic change of element size (default - false). Does not affect font size.
- `multiline` - allows display of multiline text.
- `text-wrap` - allows automatic text wrapping (works only with multiline: "true")
- `editable` - determines whether the text can be edited.
- `line-numbers` - enables line numbers display.
- `error-color` - color when entering incorrect data (the text does not pass the validator check). Type: RGBA color.
- `text-color` - text color. Type: RGBA color.
- `validator` - lua function that checks text for correctness. Takes a string as input, returns true if the text is correct.
- `onup` - lua function called when the up arrow is pressed.
- `ondown` - lua function called when the down arrow is pressed.
- `syntax` - syntax highlighting ("lua" - Lua).
- `markup` - text markup language ("md" - Markdown).

## *trackbar*

- `min` - minimal value. Type: number. Default: 0
- `max` - maximal value. Type: number. Default: 1
- `value` - initial value. Type: number. Default: 0
- `step` - track step size. Type: number: Default: 1
- `track-width` track pointer width (in steps). Type: number. Default: 1
- `track-color` - the color of the pointer when hovering over the cursor. Type: RGBA color.
- `consumer` - Lua function - new value consumer
- `sub-consumer` - Lua function that receives intermediate values ​​(use to update text when `change-on-release="true"`)
- `supplier` - Lua function - value supplier
- `change-on-release` - Call consumer on trackbar release only. Type: boolean. Default: false

## Inline frame - *iframe*

Container for embedding an external document. Content is scaling to the iframe size.

- `src` - document id in the format `pack:name` (`pack/layouts/name.xml`)

## *select*

Drop-down list. Options are described by `option` sub-elements, the `value` attribute of which contains the value, the inner text is the text displayed in the UI.

Example of list description:

```xml
<select selected="entity" width="200"
        onselect="function(opt) print(opt) end">
    <option value="block">Block</option>
    <option value="item">Item</option>
    <option value="entity">Entity</option>
</select>
```

- `width` - minimum content width. Default: 100.
- `selected` - initially selected value. Default: "".
- `onselect` - function to which the user-selected value is passed

# Inventory elements

## *inventory*

Element is a container. Does not have specific attributes.

> [!WARNING]
> Inventories position is controlled by the engine and can not be changed by attributes *pos* and *margin*

## *slot*

Element must be in direct sub-element of *inventory*.
- `index` - inventory slot index (starting from 0). Type: integer
- `item-source` - content access panel behaviour (infinite source of an item). Type: boolean
- `sharefunc` - Lua event called on <btn>LMB</btn> + <btn>Shift</btn>. Inventory id and slot index passed as arguments.
- `updatefunc` - Lua event called on slot content update.Inventory id and slot index passed as arguments.
- `onrightclick` - Lua event called on <btn>RMB</btn> click. Inventory id and slot index passed as arguments.
- `taking` - the ability to take an item from a slot.
- `placing` - the ability to put an item in a slot.

## *slots-grid*

- `start-index` - inventory slot index of the first slot. Type: integer
- `rows` - number of grid rows (unnecessary if *cols* and *count* specified). Type: integer
- `cols` - number of grid columns (unnecessary if *rows* and *count* specified). Type: integer
- `count` - total number of slots in grid (unnecessary if *rows* and *cols* specified). Type: integer
- `interval` - visual slots interval. Type: number
- `padding` - grid padding (not slots interval). Type: number. (*deprecated*)
- `sharefunc` - Lua event called on <btn>LMB</btn> + <btn>Shift</btn>. Inventory id and slot index passed as arguments.
- `updatefunc` - Lua event called on slot content update.Inventory id and slot index passed as arguments.
- `onrightclick` - Lua event called on <btn>RMB</btn> click. Inventory id and slot index passed as arguments.
- `taking` - the ability to take an item from a slot.
- `placing` - the ability to put an item in a slot.
