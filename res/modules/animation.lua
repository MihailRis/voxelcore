local this = {}

local function bezier(a, b, c, d, u)
    local s = 1 - u
    return s * s * s * a +
        3 * s * s * u * b +
        3 * s * u * u * c +
        u * u * u * d
end

local function bezier_derivative(a, b, c, d, u)
    local s = 1 - u
    return
        3 * s * s * (b - a) +
        6 * s * u * (c - b) +
        3 * u * u * (d - c)
end

function this.bezier_interpolation(k0, k1, t)
    local frame = k0.frame + t * (k1.frame - k0.frame)

    local u = t

    for i=1,8 do
        local x = bezier(
            k0.frame,
            k0.rx,
            k1.lx,
            k1.frame,
            u
        )

        local dx = bezier_derivative(
            k0.frame,
            k0.rx,
            k1.lx,
            k1.frame,
            u
        )
        if math.abs(dx) < 1e-8 then
            break
        end

        u = u - (x - frame) / dx

        if u < 0 then u = 0 end
        if u > 1 then u = 1 end
    end

    return bezier(k0.value, k0.ry, k1.ly, k1.value, u)
end

return this
