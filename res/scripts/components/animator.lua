local animation_util = require "core:animation"

local tsf = entity.transform
local rig = entity.skeleton
local body = entity.rigidbody

local head_idx = rig:index("head")
local body_idx = rig:index("body")

local leg_left = rig:index("leg_left")
local leg_right = rig:index("leg_right")

local leg_left_btm = rig:index("leg_left_btm")
local leg_right_btm = rig:index("leg_right_btm")

local hand_left = rig:index("hand_left")
local hand_right = rig:index("hand_right")

local hand_left_btm = rig:index("hand_left_btm")
local hand_right_btm = rig:index("hand_right_btm")


local tm_offset = random.random(10)
local prev_speed = 0.0

local CH_TRANSLATE = 1
local CH_ROTATE = 2

local AX_X = 1
local AX_Y = 2
local AX_Z = 3

local INT_CONST = 1
local INT_LINEAR = 2
local INT_BEZIER = 3

local patterns =  {
    {name="sint", pattern="sin(t)"},
    {name="sint2", pattern="sin(t * 2)"},
}

local function process_expression(src, memoised)
    for i, pattern in ipairs(patterns) do
        local pattern_safe = string.pattern_safe(pattern.pattern)
        if src:find(pattern_safe) then
            memoised[pattern.name] = pattern.pattern
            src = src:gsub(pattern_safe, pattern.name)
        end
    end
    return src
end

local function key_neighbors(keys, frame)
    local left = 1
    local right = #keys

    while left <= right do
        local mid = math.floor((left + right) / 2)

        if keys[mid].frame < frame then
            left = mid + 1
        elseif keys[mid].frame > frame then
            right = mid - 1
        else
            return mid, mid
        end
    end
    if left > #keys then
        left = #keys
    end
    return right, left
end

local env = {
    mat4 = mat4,
    X = {1, 0, 0},
    Y = {0, 1, 0},
    Z = {0, 0, 1},
    DST = mat4.idt(),
    value_at = function(keys, frame, interp)
        local left, right = key_neighbors(keys, frame)
        if left == right then
            return keys[left].value
        end
        left = keys[left]
        if interp == INT_CONST then
            return left.value
        end
        right = keys[right]
        local t = (frame - left.frame) / (right.frame - left.frame)
        if interp == INT_BEZIER then
            return animation_util.bezier_interpolation(left, right, t)
        end
        return left.value * (1.0 - t) + right.value * t
    end,
}
table.extend(env, math)

local function codegen_track(lines, memoised, keysets)
    local code = ""
    local translation = {false, false, false}
    local rotation = {false, false, false}
    for i, line in ipairs(lines) do
        if line.expression then
            code = code .. "\n  local l" .. i .. " = (" .. process_expression(line.expression, memoised) .. ")"
        elseif line.keys then
            keysets[i] = line.keys
            code = code .. "\n  local l" .. i .. " = value_at(keysets["..i.."], t * 3 % 60, " .. line.interp .. ")"
        end

        if line.channel == CH_TRANSLATE then
            translation[line.axis] = i
        elseif line.channel == CH_ROTATE then
            rotation[line.axis] = i
        end
    end

    code = code .. "\n  mat4.idt(dst)"
    if translation[1] or translation[2] or translation[3] then
        code = code .. "\n  mat4.translate(dst, {" ..
        (translation[1] and ("l" .. translation[1]) or '0').. ", " ..
        (translation[2] and ("l" .. translation[2]) or '0').. ", " ..
        (translation[3] and ("l" .. translation[3]) or '0').. "}, dst)"
    end

    local axis_names = {"X", "Y", "Z"}
    for axis, var in ipairs(rotation) do
        if var then
            code = code .. "\n  mat4.rotate(dst, " .. axis_names[axis] .. ", l" ..  var .. ", dst)"
        end
    end

    return code
end

local function compile_track(linesets)
    local code = ""
    local memoised = {}
    local keysets = {}

    for i, lineset in ipairs(linesets) do
        local lineset_code = codegen_track(lineset.lines, memoised, keysets)
        code = code .. "\n do" .. lineset_code .. "\n end\n" ..
            " rig:set_matrix(" .. lineset.bone_index .. ", dst)\n"
            -- " blank(" .. lineset.bone_index .. ", dst)\n"
    end

    local memoised_code = ""
    for name, expression in pairs(memoised) do
        memoised_code = memoised_code .. "\n local " .. name .. " = " .. expression
    end

    if #memoised_code > 0 then
        code = memoised_code .. "\n" .. code
    end

    local src = "return function(rig, t, m)\n local dst = DST\n" .. code .. "\nend"
    print(src)
    local generator, err = load(src, "<expr>", "bt", table.extend({keysets = keysets}, env))
    if not generator then
        error(err)
    end
    return generator()
end

local linesets = {
    {bone_index=body_idx, lines={{
        channel = CH_TRANSLATE,
        axis = AX_Y,
        expression = "sin(t * 2) * 0.1 * m"
    }}},
    {bone_index=leg_left, lines={{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "sin(t) * 45 * m"
    }}},
    {bone_index=leg_right, lines={{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "-sin(t) * 45 * m"
    }}},
    {bone_index=leg_left_btm, lines={{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "(-sin(t * 2) * 45 - 45) * m"
    }}},
    {bone_index=leg_right_btm, lines={{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "(-sin(t * 2) * 45 - 45) * m"
    }}},
    {bone_index=hand_left, lines={{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "-sin(t) * 45 * m"
    }}},
    {bone_index=hand_right, lines={{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "sin(t) * 45 * m"
    }}},
    {bone_index=hand_left_btm, lines={{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "(sin(t * 2) * 45 + 45) * m"
    }}},
    {bone_index=hand_right_btm, lines={{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "(sin(t * 2) * 45 + 45) * m"
    }}},

    {bone_index=head_idx, lines={{
        channel = CH_TRANSLATE,
        axis = AX_X,
        interp = INT_BEZIER,
        keys = {
            {
                frame = 0,
                value = 2.37319803237915,
                lx = -18.666540145874,
                ly = 2.3008770942688,
                rx = 57.5497436523438,
                ry = 2.58115172386169,
            }, {
                frame = 60,
                value = -2.95866346359253,
                lx = 38.4764099121094,
                ly = 9.56476593017578,
                rx = 76.9985656738281,
                ry = -12.8492240905762,
            }
        }
    },{
        channel = CH_TRANSLATE,
        axis = AX_Z,
        interp = INT_BEZIER,
        keys = {
            {
                frame = 0,
                value = -0.65947163105011,
                lx = -18.6666660308838,
                ly = -0.65947163105011,
                rx = 20.6666660308838,
                ry = -0.65947163105011,
            }, {
                frame = 60,
                value = 1.03530943393707,
                lx = 38.4764099121094,
                ly = 13.5587406158447,
                rx = 76.9985809326172,
                ry = -8.85525798797607,
            }
        }
    },{
        channel = CH_TRANSLATE,
        axis = AX_Y,
        interp = INT_BEZIER,
        keys = {
            {
                frame = 0,
                value = 0.697231292724609,
                lx = -18.6666660308838,
                ly = 0.697231292724609,
                rx = 20.6666660308838,
                ry = 0.697231292724609,
            }, {
                frame = 60,
                value = -0.432764053344727,
                lx = 38.4764099121094,
                ly = 12.0906667709351,
                rx = 76.9985809326172,
                ry = -10.3233318328857,
            }
        }
    }}},
}

local track = compile_track(linesets)

function on_render()
    local tm = time.uptime() * 10 + tm_offset
    local speed = vec3.length(body:get_vel()) / 7.0
    local delta = time.delta()

    prev_speed = prev_speed * (1.0 - delta * 10) + speed * delta * 10
    speed = prev_speed

    -- local ttm = time.precise_time()
    -- local matrix = mat4.idt()
    track(rig, tm, speed)
    -- for i, track in ipairs(tracks) do
        -- rig:set_matrix(track.bone_index, track.generator(matrix, tm, speed))
    -- end

    -- rig:set_matrix(body_idx, mat4.translate({0, math.sin(tm * 2) * 0.1 * speed, 0}))

    -- rig:set_matrix(leg_left, mat4.rotate({1, 0, 0}, (math.sin(tm) * 45) * speed))
    -- rig:set_matrix(leg_right, mat4.rotate({1, 0, 0}, (-math.sin(tm) * 45) * speed))

    -- rig:set_matrix(leg_left_btm, mat4.rotate({1, 0, 0}, (-math.sin(tm * 2) * 45 - 45) * speed))
    -- rig:set_matrix(leg_right_btm, mat4.rotate({1, 0, 0}, (-math.sin(tm * 2) * 45 - 45) * speed))

    -- rig:set_matrix(hand_left, mat4.rotate({1, 0, 0}, (-math.sin(tm) * 45) * speed))
    -- rig:set_matrix(hand_right, mat4.rotate({1, 0, 0}, (math.sin(tm) * 45) * speed))

    -- rig:set_matrix(hand_left_btm, mat4.rotate({1, 0, 0}, (math.sin(tm * 2) * 45 + 45) * speed))
    -- rig:set_matrix(hand_right_btm, mat4.rotate({1, 0, 0}, (math.sin(tm * 2) * 45 + 45) * speed))
    -- print(math.floor((time.precise_time() - ttm) * 1e6), "mcs")
end
