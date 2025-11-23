-- --------- Deprecated functions ------ --
local function wrap_deprecated(func, name, alternatives)
    return function (...)
        on_deprecated_call(name, alternatives)
        return func(...)
    end
end

block_index = wrap_deprecated(block.index, "block_index", "block.index")
block_name = wrap_deprecated(block.name, "block_name", "block.name")
blocks_count = wrap_deprecated(block.defs_count, "blocks_count", "block.defs_count")
is_solid_at = wrap_deprecated(block.is_solid_at, "is_solid_at", "block.is_solid_at")
is_replaceable_at = wrap_deprecated(block.is_replaceable_at, "is_replaceable_at", "block.is_replaceable_at")
set_block = wrap_deprecated(block.set, "set_block", "block.set")
get_block = wrap_deprecated(block.get, "get_block", "block.get")
get_block_X = wrap_deprecated(block.get_X, "get_block_X", "block.get_X")
get_block_Y = wrap_deprecated(block.get_Y, "get_block_Y", "block.get_Y")
get_block_Z = wrap_deprecated(block.get_Z, "get_block_Z", "block.get_Z")
get_block_states = wrap_deprecated(block.get_states, "get_block_states", "block.get_states")
set_block_states = wrap_deprecated(block.set_states, "set_block_states", "block.set_states")
get_block_rotation = wrap_deprecated(block.get_rotation, "get_block_rotation", "block.get_rotation")
set_block_rotation = wrap_deprecated(block.set_rotation, "set_block_rotation", "block.set_rotation")
get_block_user_bits = wrap_deprecated(block.get_user_bits, "get_block_user_bits", "block.get_user_bits")
set_block_user_bits = wrap_deprecated(block.set_user_bits, "set_block_user_bits", "block.set_user_bits")

function load_script(path, nocache)
    on_deprecated_call("load_script", "require or loadstring")
    return __load_script(path, nocache)
end

_dofile = dofile
-- Replaces dofile('*/content/packid/*') with load_script('packid:*') 
function dofile(path)
    on_deprecated_call("dofile", "require or loadstring")
    local index = string.find(path, "/content/")
    if index then
        local newpath = string.sub(path, index+9)
        index = string.find(newpath, "/")
        if index then
            local label = string.sub(newpath, 1, index-1)
            newpath = label..':'..string.sub(newpath, index+1)
            if file.isfile(newpath) then
                return __load_script(newpath, true)
            end
        end
    end
    return _dofile(path)
end
