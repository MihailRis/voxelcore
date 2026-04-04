function generate_heightmap(x, y, w, h, s, inputs)
    local map = Heightmap(w, h)
    map:add(0.1)
    return map
end
