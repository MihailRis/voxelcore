# Console

To work with the command interpreter, use the **console** library.

When sending a command via the standard console (core:console layout):
1. the `allow-cheats` rule is checked
2. the `player`, `pos.x|y|z`, `entity.id`, `entity.selected` variables are automatically set.
3. the command handler is called - console.submit or the default one.

The default handler calls console.execute, passing the result to the console.log call.

## Commands creation

To create a console command, use the following function:

```python
console.add_command(scheme: str, executor: function)
```

Scheme has the following syntax:

```
name positional arguments {keyword arguments}
```

Name may contain:
- latin letters
- digits (except the first character)
- `.`, `_`, `-`

Positional arguments are separated by spaces and have the following syntax:

```
name:type                        (option 1)
name:type=default                (option 2)
name:type~origin                 (option 3)
name:type=default~origin         (option 4)
```

Available types:
- **int** - integer
- **num** - floating-point number
- **str** - string
- **sel** - selector (object id represented by an integer)
- **enum** - enumeration

Options 3 and 4 show the `~` operator that allows you to use relative values. *Origin* - the value relative to which the user will be specified. For example, the player's position.

The relative operator only works with numbers (num or int)

Variables assigned via **console.set** can be specified as origin values.

Example:

```python
x:num~pos.x
```

Variables may be specified as default values using the `$` prefix:

```python
t:int=$time
```

Enumerations are declared the following way:

```python
mode:[replace|destruct|none]
```

Or with a variable:

```python
mode:enum $modes
```

Selectors are specified with the `@` prefix. At the moment they are unused due to the lack of an object model. Should be made optional and use variables:

```python
obj:sel=$obj.id # obj.id - player id
```

Keyword arguments are specified in a special block, delimited by curly braces `{ }`, following the same pattern as positional arguments.

Example:

```python
eval name:str="World" {greeting:str='Hello'}
```

## Command scheme examples

Schemes of standard commands can be found in the file `res/script/stdcmd.lua`.

Example - command **tp**:

```python
tp obj:sel=$obj.id x:num~pos.x y:num~pos.y z:num~pos.z
```

Full lua code of the command creation:

```lua
console.add_command(
    "tp obj:sel=$obj.id x:num~pos.x y:num~pos.y z:num~pos.z",
    "Teleport object",
    function (args, kwargs)
        player.set_pos(unpack(args))
    end
)
```

- Checked values of positional arguments are passed to **args**.
- A table of keyword argument values is passed to **kwargs**.

The command interpreter performs type checking and casting automatically.
