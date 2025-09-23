# Item properties

## Visual

### *icon-type* and *icon* itself

Icon type defines a source of an item image displayed in inventory.
- **none** - invisible type, used for *core:empty* only (empty item, like the air block). May be removed in future updates.
- **sprite** - default type. 2D image. Requires *icon* set to *atlas_name:texture_name*. Example: *blocks:notfound*.
  There's two atlases available:
	- **blocks** (generated from *png* files in *res/textures/blocks/*)
	- **items** (generated from *png* files in *res/textures/items/*)
- **block** - block preview. Block ID must be specified in **icon** property. Example: *base:wood*.

### Item model - `model-name`

Name of the item model. The model will be loaded automatically.
Default value is `packid:itemname.model`.
If the model is not specified, an automatic one will be generated.

### Caption and Description
`caption` - name of item in inventory
`description` - item description in inventory

this props allow to use `md`

*see [Text Styles](/doc/en/text-styles.md)*

## Behaviour

### *placing-block*

Specifies what block will be placed on RMB click. Automatically specified in generated items.

Example: an items that places bazalt blocks:

```json
"placing-block": "base:bazalt"
```

### *emission*

Light emitted when player holds the item in hand.

An array of 3 integers - R, G, B of light in range \[0, 15\]

Examples:

- *\[15, 15, 15\]* - white with maximal intensity
- *\[7, 0, 0\]* - dim red light
- *\[0, 0, 0\]* - no emission (default value)

### *stack-size*

Maximal number of an item units in one slot. Default - 64.

### Number of uses (durability) - `uses`

Property used via [inventory.use](scripting/builtins/libinventory.md).

Property status is displayed in the inventory interface. Display method is defined via `uses-display`.

### Display of uses - `uses-display`

- `none` - display disabled
- `number` - number
- `relation` - current value to initial value (x/y)
- `vbar` - vertical scale (used by default)

## Tags

Tags allow you to designate general properties of items. Names should be formatted as `prefix:tag_name`.
The prefix is ​​optional, but helps avoid unwanted logical collisions. Example:

```json
{
    "tags": [
        "core:fuel",
        "base_survival:poison",
    ]
}
```

Tags can also be added to items from other packs using the `your_pack:tags.toml` file. Example

```toml
"prefix:tag_name" = [
    "random_pack:item",
    "another_pack:some_block",
]
"other_prefix:other_tag_name" = [
    # ...
]
```
