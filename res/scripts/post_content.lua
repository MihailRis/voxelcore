local user_props = file.read_combined_object("config/user-props.toml")
local names = {
    "parent", "caption", "texture", "texture-faces", "model", "model-name",
    "model-primitives", "material", "rotation", "hitboxes", "hitbox", "emission",
    "size", "obstacle", "replaceable", "light-passing", "sky-light-passing",
    "shadeless", "ambient-occlusion", "breakable", "selectable", "grounded",
    "hidden", "draw-group", "picking-item", "surface-replacement", "script-name",
    "ui-layout", "inventory-size", "tick-interval", "overlay-texture",
    "translucent", "fields", "particles", "icon-type", "icon", "placing-block", 
    "stack-size", "name", "script-file"
}
for name, _ in pairs(user_props) do
    table.insert(names, name)
end
-- remove undefined properties
for id, blockprops in pairs(block.properties) do
    for propname, value in pairs(blockprops) do
        if not table.has(names, propname) then
            blockprops[propname] = nil
        end
    end
end
for id, itemprops in pairs(item.properties) do
    for propname, value in pairs(itemprops) do
        if not table.has(names, propname) then
            itemprops[propname] = nil
        end
    end
end

local function make_read_only(t)
    setmetatable(t, {
        __newindex = function()
            error("table is read-only")
        end
    })    
end

make_read_only(block.properties)
for k,v in pairs(block.properties) do
    make_read_only(v)
end

local function cache_names(library)
    local indices = {}
    local names = {}
    for id=0,library.defs_count()-1 do
        local name = library.properties[id].name
        indices[name] = id
        names[id] = name
    end

    function library.name(id)
        return names[id]
    end

    function library.index(name)
        return indices[name]
    end
end

cache_names(block)
cache_names(item)

local scripts_registry = require "core:internal/scripts_registry"
scripts_registry.build_registry()
