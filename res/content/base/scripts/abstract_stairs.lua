-- variants
local DEFAULT = 0
local INNER_LEFT = 1
local INNER_RIGHT = 2
local OUTER_LEFT = 3
local OUTER_RIGHT = 4

local function get_stairs_info(id, x, y, z)
    if block.get(x, y, z) ~= id then
        return nil
    end
    local zx, _, zz = block.get_Z(x, y, z)
    local _, half = block.get_Y(x, y, z)
    return -zx, -zz, half
end

local function detect_variant(id, x, y, z)
    local bx, bz, half = get_stairs_info(id, x, y, z)
    local lx, _, lz = block.get_X(x, y, z)
    lx, lz = -lx, -lz

    local function is_oriented_same(ox, oz)
        local nbx, nbz, nhalf = get_stairs_info(id, x + ox, y, z + oz)
        return nbx == bx and nbz == bz and nhalf == half
    end

    local function corner_side(ox, oz, sign)
        local nbx, nbz, nhalf = get_stairs_info(id, x + ox, y, z + oz)
        if nbx == nil or nhalf ~= half or nbx * bx + nbz * bz ~= 0 then
            return nil
        end
        if is_oriented_same(nbx * sign, nbz * sign) then
            return nil
        end
        return nbx == lx and nbz == lz
    end

    local left = corner_side(bx, bz, -1)
    if left ~= nil then
        return left and OUTER_LEFT or OUTER_RIGHT
    end
    left = corner_side(-bx, -bz, 1)
    if left ~= nil then
        return left and INNER_LEFT or INNER_RIGHT
    end
    return DEFAULT
end

local function refresh(x, y, z, keep_variant)
    local variant = detect_variant(block.get(x, y, z), x, y, z)
    if keep_variant and variant == DEFAULT then
        return
    end
    if block.get_variant(x, y, z) ~= variant then
        block.set_variant(x, y, z, variant)
    end
end

function on_placed(x, y, z)
    refresh(x, y, z, false)
end

function on_update(x, y, z)
    refresh(x, y, z, true)
end
