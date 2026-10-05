local app = __vc_app
local internals = __vc_internals

local user_props = file.read_combined_object("config/user-props.toml")
local names = {
    "parent", "caption", "texture", "texture-faces", "model", "model-name",
    "model-primitives", "material", "rotation", "hitboxes", "hitbox", "emission",
    "size", "obstacle", "replaceable", "light-passing", "sky-light-passing",
    "shadeless", "ambient-occlusion", "breakable", "selectable", "grounded",
    "hidden", "draw-group", "picking-item", "surface-replacement", "script-name",
    "ui-layout", "inventory-size", "tick-interval", "overlay-texture",
    "translucent", "fields", "particles", "icon-type", "icon", "placing-block",
    "stack-size", "name", "script-file", "culling", "solid"
}
for name, _ in pairs(user_props) do
    table.insert(names, name)
end

-- remove undefined properties and build tags set
local function process_properties(lib)
    for id, props in pairs(lib.properties) do
        if props.parent then
            local parent_props = lib.properties[block.index(props.parent)] or {}
            for propname, value in pairs(parent_props) do
                if not props[propname] then
                    props[propname] = value
                end
            end
        end
        for propname, _ in pairs(props) do
            if not table.has(names, propname) then
                props[propname] = nil
            end
        end

        props.tags_set = lib.__get_tags(id)
    end
end

process_properties(block)
process_properties(item)

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

    function library.has_tag(id, tag)
        if id == nil then
            error("id is nil")
        end
        local props = library.properties[id]
        local tags_set = props.tags_set
        if tags_set then
            return tags_set[tag]
        else
            return false
        end
    end
end

cache_names(block)
cache_names(item)

__vc_scripts_registry.build_registry()

local packs = app.get_content()
table.insert(packs, 1, "core")

for i, packid in ipairs(packs) do
    local loaders_dir = packid..":scripts/loaders"
    if not file.isdir(loaders_dir) then
        goto continue
    end
    local files = file.list(loaders_dir)
    for _, path in ipairs(files) do
        if file.ext(path) == "json" then
            debug.log("registering loader "..path)
            local dto = json.parse(file.read(path))
            local module = setmetatable({}, {__index=internals.get_pack_env(packid)})
            local module_path = file.join(file.parent(path), file.stem(path)..".lua")
            if not file.exists(module_path) then
                error("loader "..path.." missing module "..module_path)
            end
            load(file.read(module_path), module_path, "t", module)()
            if rawget(module, "execute") == nil then
                error("loader "..path.." missing function 'execute'")
            end
            if dto.type == "compiler" then
                internals.register_compiler(packid, dto.extensions, module)
            end
        end
    end
    ::continue::
end
