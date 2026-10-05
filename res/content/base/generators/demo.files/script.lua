local _, dir = parse_path(__DIR__)
local ores = require "base:generation/ores"
math.randomseed(SEED)
ores.load(dir)

local function get_rand(seed, x, y, z)
    local h = bit.bxor(bit.bor(x * 23729, y % 16786), y * x + seed)
    h = bit.bxor(h, z * 47917)
    h = bit.bxor(h, bit.bor(z % 12345, x + y))

    local n = (h % 10000) / 10000.0

    return n
end

local function gen_parameters(size, seed, x, y)
    local res = {}
    local rand = 0

    for i=1, size do
        rand = get_rand(seed, x, y, rand)
        table.insert(res, rand)
    end

    return res
end

function place_structures(x, z, w, d, hmap, chunk_height)
    local placements = {}
    ores.place(placements, x, z, w, d, SEED, hmap, chunk_height)
    return placements
end

function place_structures_wide(x, z, w, d, chunk_height)
    local placements = {}
    local rands = gen_parameters(11, SEED, x, z)
    if rands[1] < 0.05 then -- generate caves

        local sx = x + rands[2] * 10 - 5
        local sy = rands[3] * (chunk_height / 4) + 10
        local sz = z + rands[4] * 10 - 5

        local dir = rands[5] * math.pi * 2
        local dir_inertia = (rands[6] - 0.5) * 2
        local elevation = -3
        local width = rands[7] * 3 + 2

        for i=1,18 do
            local dx = math.sin(dir) * 10
            local dz = -math.cos(dir) * 10

            local ex = sx + dx
            local ey = sy + elevation
            local ez = sz + dz

            table.insert(placements, 
                {":line", 0, {sx, sy, sz}, {ex, ey, ez}, width})

            sx = ex
            sy = ey
            sz = ez

            dir_inertia = dir_inertia * 0.8 + 
                (rands[8] - 0.5) * math.pow(rands[9], 2) * 8
            elevation = elevation * 0.9 + 
                (rands[10] - 0.4) * (1.0-math.pow(rands[11], 4)) * 8
            dir = dir + dir_inertia
        end
    end
    return placements
end

function generate_heightmap(x, y, w, h, s, inputs)
    local umap = Heightmap(w, h)
    local vmap = Heightmap(w, h)
    umap.noiseSeed = SEED
    vmap.noiseSeed = SEED
    vmap:noise({x+521, y+70}, 0.1*s, 3, 25.8)
    vmap:noise({x+95, y+246}, 0.15*s, 3, 25.8)

    local map = Heightmap(w, h)
    map.noiseSeed = SEED
    map:noise({x, y}, 0.8*s, 4, 0.02)
    map:cellnoise({x, y}, 0.1*s, 3, 0.3, umap, vmap)
    map:add(0.7)

    local rivermap = Heightmap(w, h)
    rivermap.noiseSeed = SEED
    rivermap:noise({ x + 21, y + 12 }, 0.1 * s, 4)
    rivermap:pow(4)
    rivermap:mul(3500)
    rivermap:add(1)
    rivermap:pow(-1)
    rivermap:mul(-1)
    rivermap:add(1)

    local rivermap_bottom = Heightmap(w, h)
    rivermap_bottom:add(0.23)
    rivermap_bottom:noise({ x, y }, 2 * s, 4, 0.005)

    map:sub(rivermap_bottom)

    local map_multiplied = Heightmap(w, h)
    map_multiplied:add(map)
    map_multiplied:mul(rivermap)

    map:min(map_multiplied)

    map:add(rivermap_bottom)

    local desertmap = Heightmap(w, h)
    desertmap.noiseSeed = SEED
    desertmap:cellnoise({x+52, y+326}, 0.3*s, 2, 0.2)
    desertmap:add(0.5)
    map:mixin(desertmap, inputs[1])
    return map
end

function generate_biome_parameters(x, y, w, h, s)
    local tempmap = Heightmap(w, h)
    tempmap.noiseSeed = SEED + 5324
    tempmap:noise({x, y}, 0.08*s, 6)
    tempmap:mul(0.5)
    tempmap:add(0.5)
    local hummap = Heightmap(w, h)
    hummap.noiseSeed = SEED + 953
    hummap:noise({x, y}, 0.08*s, 6)
    tempmap:pow(3)
    hummap:pow(3)
    return tempmap, hummap
end
