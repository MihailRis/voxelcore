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
