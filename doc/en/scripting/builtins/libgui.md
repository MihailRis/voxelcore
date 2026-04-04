# *gui* library

The library contains functions for accessing the properties of UI elements. Instead of gui, you should use an object wrapper that provides access to properties through the __index, __newindex meta methods:

Example:

```lua
print(document.some_button.text) -- where 'some_button' is an element id
document.some_button.text = "new text"
```

```lua
-- Returns translated text.
gui.str(text: str, context: str) -> str

-- Returns size of the main container (window).
gui.get_viewport() -> {int, int}

-- Returns environment (global variables table) of the specified document.
gui.get_env(document: str) -> table
```

```lua
-- Returns information about all loaded locales (res/texts/*).
gui.get_locales_info() -> table of tables {
  name: str
 }
-- where
--   key - locale id following isolangcode_ISOCOUNTRYCODE format
--   value - table {
--       name: str # locale display name
--   }
```

## Frames

```lua
-- A replacement for menu:reset() to close the pause menu, deactivating the main UI frame.
gui.close_menu()

-- Creates a frame.
gui.create_frame(
    -- Global frame id (not related to the UI 'id' property).
    id: str,
    -- The texture to render the frame to.
    -- If the string is empty, the frame is rendered to the screen.
    output_texture: str,
    -- Frame size. For example: {640, 480}
    size: vec2
) -> Element, Document

-- Returns the id of the active frame (not the element id).
gui.get_active_frame() -> str

-- Sets the active frame, receiving user input.
-- An empty string specifies the null frame capturing the cursor.
gui.set_active_frame(
    -- ID of the frame created via gui.create_frame
    id: str,
    -- Function providing the cursor position in the frame.
    -- Used for custom projection (e.g., in 3D)
    [optional] cursorLocator: function() -> number, number
)
```

## Markup

```lua
-- Removes markup from text.
gui.clear_markup(
    -- markup language ("md" - Markdown)
    language: str,
    -- text with markup
    text: str
) -> str

-- Escapes markup in text.
gui.escape_markup(
    -- markup language ("md" - Markdown)
    language: str,
    -- text with markup
    text: str
) -> str
```


## Dialog windows

```lua
-- Displays a message box. Doesn't stop code execution.
gui.show_message(
    message: string -- message (not automatically translated, use gui.str(...))
    on_ok: function() -> nil -- called when closing
)

-- Asks for confirmation of the action. Doesn't stop code execution.
gui.ask(
    -- message (not automatically translated, use gui.str(...))
    message: string,
    -- function called on confirmation
    on_confirm: function() -> nil,
    -- function called on rejection/cancellation
    [optional] on_deny: function() -> nil,
    -- confirm button text (default: "Yes")
    -- use an empty string for the default value if you want to specify no_text.
    [optional] yes_text: string
    -- reject button text (default: "No")
    [optional] no_text: string
)
```

## Dialog menu pages (legacy)

```lua
-- Displays a message box. Non-blocking.
gui.alert(
    -- message (not automatically translated, use gui.str(...))
    message: str,
    -- function called on close
    on_ok: function() -> nil
)

-- Requests confirmation from the user for an action. Non-blocking.
gui.confirm(
    -- message (does not translate automatically, use gui.str(...))
    message: str,
    -- function called upon confirmation
    on_confirm: function() -> nil,
    -- function called upon denial/cancellation
    [optional] on_deny: function() -> nil,
    -- text for the confirmation button (default: "Yes")
    -- use an empty string for the default value if you want to specify no_text.
    [optional] yes_text: str,
    -- text for the denial button (default: "No")
    [optional] no_text: str,
)
```

## Documents and templates

```lua
-- Loads a UI document with its script, returns the name of the document if successfully loaded.
gui.load_document(
    -- Path to the xml file of the page. Example: `core:layouts/pages/main.xml`
    path: str,
    -- Name (id) of the document. Example: `core:pages/main`
    name: str
    -- Table of parameters passed to the on_open event
    args: table
) -> str

-- Loads a template into the layout
gui.template(
    -- template name in /layouts/templates without path and extension
    name: str,
    -- variable table (can be used in markup)
    -- * Ex: <label>%{text}</label>
    -- * text in this case is the value from params with the text key
    params: table,
    -- table, available in events as the global variable DATA
    [optional] data: table
) -> str
```

## Root document

```lua
-- Root UI document
gui.root: Document
```
