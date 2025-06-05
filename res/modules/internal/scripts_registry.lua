local export = {
    filenames = {},
    registry = {}
}

local function collect_components(dirname, dest)
    if file.isdir(dirname) then
        local files = file.list(dirname)
        for i, filename in ipairs(files) do
            if file.ext(filename) == "lua" then
                table.insert(dest, filename)
                export.registry[filename] = {
                    type="component", 
                    unit=file.prefix(filename)..":"..file.stem(filename)
                }
            end
        end
    end
end

local function collect_scripts(dirname, dest, ismodule)
    if file.isdir(dirname) then
        local files = file.list(dirname)
        for i, filename in ipairs(files) do
            if file.name(filename) == "components" and not ismodule then
                collect_components(filename, dest)
            elseif file.isdir(filename) then 
                collect_scripts(filename, dest)
            elseif file.ext(filename) == "lua" then
                table.insert(dest, filename)
            end
        end
    end
end

local function load_scripts_list(packs)
    local registry = export.registry
    for _, packid in ipairs(packs) do
        collect_scripts(packid..":modules", export.filenames, true)
    end
    for _, filename in ipairs(export.filenames) do
        registry[filename] = {
            type="module", 
            unit=file.join(file.parent(file.prefix(filename)..":"..
                           filename:sub(filename:find("/")+1)),
                                file.stem(filename))
        }
    end
    for _, packid in ipairs(packs) do
        collect_scripts(packid..":scripts", export.filenames, false)
    end
end

local function load_models_list(packs)
    local registry = export.registry
    for _, filename in ipairs(file.list("models")) do
        local ext = file.ext(filename)
        if ext == "xml" or ext == "vcm" then
            registry[filename] = {type="model", unit=file.stem(filename)}
            table.insert(export.filenames, filename)
        end
    end
end

function export.build_registry()
    local registry = {}
    for id, props in pairs(block.properties) do
        registry[props["script-file"]] = {type="block", unit=block.name(id)}
    end
    for id, props in pairs(item.properties) do
        registry[props["script-file"]] = {type="item", unit=item.name(id)}
    end
    local packs = pack.get_installed()
    for _, packid in ipairs(packs) do
        registry[packid..":scripts/world.lua"] = {type="world", unit=packid}
        registry[packid..":scripts/hud.lua"] = {type="hud", unit=packid}
    end
    export.registry = registry
    export.filenames = {}

    load_scripts_list(packs)
    load_models_list(packs)
end

function export.get_info(filename)
    return export.registry[filename]
end

return export
