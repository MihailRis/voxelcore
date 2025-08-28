function on_block_broken(id, x, y, z, playerid)
    if gfx then
        local size = {block.get_size(id)}
        gfx.particles.emit({
            x + size[1] * 0.5, 
            y + size[1] * 0.5, 
            z + size[1] * 0.5
        }, 64, {
            lifetime=1.0,
            spawn_interval=0.0001,
            explosion={4, 4, 4},
            texture="blocks:"..block.get_textures(id)[1],
            random_sub_uv=0.1,
            size={0.1, 0.1, 0.1},
            spawn_shape="box",
            spawn_spread=vec3.mul(size, 0.4)
        })
    end

    rules.create("do-loot-non-player", true)
end
