# *gui* library

The library contains functions for accessing the properties of UI elements. Instead of gui, you should use an object wrapper that provides access to properties through the __index, __newindex meta methods:

Example:

```lua
print(document.some_button.text) -- where 'some_button' is an element id
document.some_button.text = "new text"
```

```python
gui.str(text: str, context: str) -> str
```

Returns translated text.

```python
gui.get_viewport() -> {int, int}
```

Returns size of the main container (window).

```python
gui.get_env(document: str) -> table
```

Returns environment (global variables table) of the specified document.

```lua
gui.get_locales_info() -> table of tables {
  name: str
 }
-- where
--   key - locale id following isolangcode_ISOCOUNTRYCODE format
--   value - table {
--       name: str # locale display name
--   }
```

Returns information about all loaded locales (res/texts/\*).

```lua
gui.clear_markup(
    -- markup language ("md" - Markdown)
    language: str,
    -- text with markup
    text: str
) -> str
```

Removes markup from text.

```lua
gui.escape_markup(
    -- markup language ("md" - Markdown)
    language: str,
    -- text with markup
    text: str
) -> str
```

Escapes markup in text.

```lua
gui.alert(
    -- message (not automatically translated, use gui.str(...))
    message: str,
    -- function called on close
    on_ok: function() -> nil
)
```

Displays a message box. **Non-blocking**.

```lua
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

Requests confirmation from the user for an action. **Non-blocking**.

```lua
gui.load_document(
    -- Path to the xml file of the page. Example: `core:layouts/pages/main.xml`
    path: str,
    -- Name (id) of the document. Example: `core:pages/main`
    name: str
    -- Table of parameters passed to the on_open event
    args: table
) --> str
```

Loads a UI document with its script, returns the name of the document if successfully loaded.

```lua
gui.root: Document
```

Root UI document
