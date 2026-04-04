local ores = {}

function ores.load(directory)
    ores.ores = file.read_combined_list(directory.."/ores.json")
end

function ores.place(placements, x, z, w, d, seed, hmap, chunk_height)
    local BLOCKS_PER_CHUNK = w * d * chunk_height
    for _, ore in ipairs(ores.ores) do
        local count = BLOCKS_PER_CHUNK / ore.rarity

        -- average count is less than 1
        local addchance = math.fmod(count, 1.0)
        if math.random() < addchance then
            count = count + 1
        end

        for i=1,count do
            local sx = math.random() * w
            local sz = math.random() * d
            local sy = math.random() * (chunk_height * 0.5) + 6
            if sy < hmap:at(sx, sz) * chunk_height - 6 then
                table.insert(placements, {ore.struct, {sx, sy, sz}, math.random()*4, -1})
            end
        end
    end
end

return ores
