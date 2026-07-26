local offsets = {}
for lx=-1,1 do
    for ly=-1,1 do
        for lz=-1,1 do
            offsets[(lx * 3 + ly) * 3 + lz] = {lx, ly, lz}
        end
    end
end

function on_random_update(x, y, z)
    local dirtid = block.index('base:dirt');
    if block.is_solid_at(x, y+1, z) then
        block.set(x, y, z, dirtid, 0)
        return
    end
    local grassblockid = block.index('base:grass_block')
    for _, offset in ipairs(offsets) do
        local nx, ny, nz = x + offset[1], y + offset[2], z + offset[3]

        if block.get(nx, ny, nz) == dirtid and
        not block.is_solid_at(nx, ny + 1, nz) then
            block.set(nx, ny, nz, grassblockid, 0)
            return
        end
    end
end
