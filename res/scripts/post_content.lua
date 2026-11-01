local app = __vc_app
local internals = __vc_internals

local user_props = file.read_combined_object("config/user-props.toml")
local core_names = {
    "parent", "caption", "texture", "texture-faces", "model", "model-name",
    "model-primitives", "material", "rotation", "hitboxes", "hitbox", "emission",
    "size", "obstacle", "replaceable", "light-passing", "sky-light-passing",
    "shadeless", "ambient-occlusion", "breakable", "selectable", "grounded",
    "hidden", "draw-group", "picking-item", "surface-replacement", "script-name",
    "ui-layout", "inventory-size", "tick-interval", "overlay-texture",
    "translucent", "fields", "particles", "icon-type", "icon", "placing-block",
    "stack-size", "name", "script-file", "culling", "solid",
    "components", "sensors", "body-type", "save", "save-skeleton-pose",
    "save-skeleton-textures", "save-body-velocity", "save-body-settings",
    "skeleton-name", "lighting-mode", "mass", "elasticity", "step-height",
    "blocking", "material", "translucent", "uv-lock"
}
for name, _ in pairs(user_props) do
    table.insert(core_names, name)
end

local function get_parent_props(lib, props)
    if not props.parent then
        return {}
    end
    if lib == entities then
        local parent_id = entities.def_index(props.parent)
        if parent_id == nil then
            return {}
        end
        return entities.properties[parent_id] or {}
    end
    if lib.index == nil then
        return {}
    end
    local parent_id = lib.index(props.parent)
    if parent_id == nil then
        return {}
    end
    return lib.properties[parent_id] or {}
end

-- remove undefined properties and build tags set
local function process_properties(lib)
    for id, props in pairs(lib.properties) do
        local parent_props = get_parent_props(lib, props)
        for propname, value in pairs(parent_props) do
            if not props[propname] then
                props[propname] = value
            end
        end
        for propname, _ in pairs(props) do
            if not table.has(core_names, propname) then
                props[propname] = nil
            end
        end

        if lib.__get_tags then
            props.tags_set = lib.__get_tags(id)
        end
    end
end

process_properties(block)
process_properties(item)
process_properties(entities)

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

make_read_only(item.properties)
for k,v in pairs(item.properties) do
    make_read_only(v)
end

make_read_only(entities.properties)
for k,v in pairs(entities.properties) do
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

local function cache_entity_names()
    local indices = {}
    local names = {}
    for id=0, entities.defs_count() - 1 do
        local name = entities.properties[id].name
        indices[name] = id
        names[id] = name
    end

    function entities.name(id)
        return names[id]
    end

    function entities.index(name)
        return indices[name]
    end
end

cache_entity_names()

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
