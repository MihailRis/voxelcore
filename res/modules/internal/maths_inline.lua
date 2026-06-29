local sin = math.sin
local cos = math.cos
local sqrt = math.sqrt
local rad = math.rad

-- =================================================== --
-- ====================== vec3 ======================= --
-- =================================================== --
function vec3.add(a, b, dst)
    local btype = type(b)
    if dst then
        if btype == "table" then
            dst[1] = a[1] + b[1]
            dst[2] = a[2] + b[2]
            dst[3] = a[3] + b[3]
        else
            dst[1] = a[1] + b
            dst[2] = a[2] + b
            dst[3] = a[3] + b
        end
        return dst
    else
        if btype == "table" then
            return {a[1] + b[1], a[2] + b[2], a[3] + b[3]}
        else
            return {a[1] + b, a[2] + b, a[3] + b}
        end
    end
end

function vec3.sub(a, b, dst)
    local btype = type(b)
    if dst then
        if btype == "table" then
            dst[1] = a[1] - b[1]
            dst[2] = a[2] - b[2]
            dst[3] = a[3] - b[3]
        else
            dst[1] = a[1] - b
            dst[2] = a[2] - b
            dst[3] = a[3] - b
        end
        return dst
    else
        if btype == "table" then
            return {a[1] - b[1], a[2] - b[2], a[3] - b[3]}
        else
            return {a[1] - b, a[2] - b, a[3] - b}
        end
    end
end

function vec3.mul(a, b, dst)
    local btype = type(b)
    if dst then
        if btype == "table" then
            dst[1] = a[1] * b[1]
            dst[2] = a[2] * b[2]
            dst[3] = a[3] * b[3]
        else
            dst[1] = a[1] * b
            dst[2] = a[2] * b
            dst[3] = a[3] * b
        end
        return dst
    else
        if btype == "table" then
            return {a[1] * b[1], a[2] * b[2], a[3] * b[3]}
        else
            return {a[1] * b, a[2] * b, a[3] * b}
        end
    end
end

function vec3.div(a, b, dst)
    local btype = type(b)
    if dst then
        if btype == "table" then
            dst[1] = a[1] / b[1]
            dst[2] = a[2] / b[2]
            dst[3] = a[3] / b[3]
        else
            dst[1] = a[1] / b
            dst[2] = a[2] / b
            dst[3] = a[3] / b
        end
        return dst
    else
        if btype == "table" then
            return {a[1] / b[1], a[2] / b[2], a[3] / b[3]}
        else
            return {a[1] / b, a[2] / b, a[3] / b}
        end
    end
end

function vec3.abs(a, dst)
    local x = a[1]
    local y = a[2]
    local z = a[3]
    if dst then
        dst[1] = x < 0.0 and -x or x
        dst[2] = y < 0.0 and -y or y
        dst[3] = z < 0.0 and -z or z
    else
        return {
            x < 0.0 and -x or x,
            y < 0.0 and -y or y,
            z < 0.0 and -z or z,
        }
    end
end

function vec3.dot(a, b)
    return a[1] * b[1] + a[2] * b[2] + a[3] * b[3]
end

function vec3.mix(a, b, t, dest)
    if dest then
        dest[1] = a[1] * (1.0 - t) + b[1] * t
        dest[2] = a[2] * (1.0 - t) + b[2] * t
        dest[3] = a[3] * (1.0 - t) + b[3] * t
        return dest
    else
        return {
            a[1] * (1.0 - t) + b[1] * t,
            a[2] * (1.0 - t) + b[2] * t,
            a[3] * (1.0 - t) + b[3] * t,
        }
    end
end

-- =================================================== --
-- ====================== vec2 ======================= --
-- =================================================== --
function vec2.add(a, b, dst)
    local btype = type(b)
    if dst then
        if btype == "table" then
            dst[1] = a[1] + b[1]
            dst[2] = a[2] + b[2]
        else
            dst[1] = a[1] + b
            dst[2] = a[2] + b
        end
        return dst
    else
        if btype == "table" then
            return {a[1] + b[1], a[2] + b[2]}
        else
            return {a[1] + b, a[2] + b}
        end
    end
end

function vec2.sub(a, b, dst)
    local btype = type(b)
    if dst then
        if btype == "table" then
            dst[1] = a[1] - b[1]
            dst[2] = a[2] - b[2]
        else
            dst[1] = a[1] - b
            dst[2] = a[2] - b
        end
        return dst
    else
        if btype == "table" then
            return {a[1] - b[1], a[2] - b[2]}
        else
            return {a[1] - b, a[2] - b}
        end
    end
end

function vec2.mul(a, b, dst)
    local btype = type(b)
    if dst then
        if btype == "table" then
            dst[1] = a[1] * b[1]
            dst[2] = a[2] * b[2]
        else
            dst[1] = a[1] * b
            dst[2] = a[2] * b
        end
        return dst
    else
        if btype == "table" then
            return {a[1] * b[1], a[2] * b[2]}
        else
            return {a[1] * b, a[2] * b}
        end
    end
end

function vec2.div(a, b, dst)
    local btype = type(b)
    if dst then
        if btype == "table" then
            dst[1] = a[1] / b[1]
            dst[2] = a[2] / b[2]
        else
            dst[1] = a[1] / b
            dst[2] = a[2] / b
        end
        return dst
    else
        if btype == "table" then
            return {a[1] / b[1], a[2] / b[2]}
        else
            return {a[1] / b, a[2] / b}
        end
    end
end

function vec2.abs(a, dst)
    local x = a[1]
    local y = a[2]
    if dst then
        dst[1] = x < 0.0 and -x or x
        dst[2] = y < 0.0 and -y or y
    else
        return {
            x < 0.0 and -x or x,
            y < 0.0 and -y or y,
        }
    end
end

function vec2.dot(a, b)
    return a[1] * b[1] + a[2] * b[2]
end

function vec2.mix(a, b, t, dest)
    if dest then
        dest[1] = a[1] * (1.0 - t) + b[1] * t
        dest[2] = a[2] * (1.0 - t) + b[2] * t
        return dest
    else
        return {
            a[1] * (1.0 - t) + b[1] * t,
            a[2] * (1.0 - t) + b[2] * t,
        }
    end
end

function mat4.idt(dst)
    if dst then
        for i=1,16 do
            dst[i] = (i % 5 == 1) and 1 or 0
        end
    end
    return {
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    }
end

function mat4.rotate(...)
    local argc = select("#", ...)

    local matrix, axis, angle, dst

    if argc == 2 then
        axis, angle = ...
        matrix = mat4.idt()
    elseif argc == 3 then
        matrix, axis, angle = ...
    elseif argc == 4 then
        matrix, axis, angle, dst = ...
    else
        error("invalid arguments number (2, 3 or 4 expected)")
    end

    angle = rad(angle)

    local x = axis[1]
    local y = axis[2]
    local z = axis[3]

    local len = sqrt(x*x + y*y + z*z)
    if len == 0 then
        error("rotation axis has zero length")
    end

    x = x / len
    y = y / len
    z = z / len

    local c = cos(angle)
    local s = sin(angle)
    local t = 1 - c

    local r00 = t*x*x + c
    local r01 = t*x*y - s*z
    local r02 = t*x*z + s*y

    local r10 = t*x*y + s*z
    local r11 = t*y*y + c
    local r12 = t*y*z - s*x

    local r20 = t*x*z - s*y
    local r21 = t*y*z + s*x
    local r22 = t*z*z + c

    local out = dst or {}

    out[1] = matrix[1]*r00 + matrix[5]*r10 + matrix[9]*r20
    out[2] = matrix[2]*r00 + matrix[6]*r10 + matrix[10]*r20
    out[3] = matrix[3]*r00 + matrix[7]*r10 + matrix[11]*r20
    out[4] = matrix[4]*r00 + matrix[8]*r10 + matrix[12]*r20

    out[5] = matrix[1]*r01 + matrix[5]*r11 + matrix[9]*r21
    out[6] = matrix[2]*r01 + matrix[6]*r11 + matrix[10]*r21
    out[7] = matrix[3]*r01 + matrix[7]*r11 + matrix[11]*r21
    out[8] = matrix[4]*r01 + matrix[8]*r11 + matrix[12]*r21

    out[9]  = matrix[1]*r02 + matrix[5]*r12 + matrix[9]*r22
    out[10] = matrix[2]*r02 + matrix[6]*r12 + matrix[10]*r22
    out[11] = matrix[3]*r02 + matrix[7]*r12 + matrix[11]*r22
    out[12] = matrix[4]*r02 + matrix[8]*r12 + matrix[12]*r22

    out[13] = matrix[13]
    out[14] = matrix[14]
    out[15] = matrix[15]
    out[16] = matrix[16]

    return out
end
