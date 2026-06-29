local tsf = entity.transform
local rig = entity.skeleton
local body = entity.rigidbody


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

local function compile_track(bone_index, lines)
    local env = {
        mat4 = mat4,
    }
    table.extend(env, math)

    local code = ""
    local translation = {false, false, false}
    local rotation = {false, false, false}
    for i, line in ipairs(lines) do
        code = code .. "\n  local l" .. i .. " = (" .. line.expression .. ")"
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

    for axis, var in ipairs(rotation) do
        if var then
            code = code .. "\n  mat4.rotate(dst, {"
            .. ((axis == 1) and '1' or '0') .. ", "
            .. ((axis == 2) and '1' or '0') .. ", "
            .. ((axis == 3) and '1' or '0') .. "}, l" ..  var .. ", dst)"
        end
    end

    local src = "return function(dst, t, m)" .. code .. "\n  return dst\nend"
    local generator, err = load(src, "<expr>", "bt", env)
    if not generator then
        error(err)
    end
    print(src)
    return {
        bone_index = bone_index,
        generator = generator()
    }
end

local tracks = {
    compile_track(body_idx, {{
        channel = CH_TRANSLATE,
        axis = AX_Y,
        expression = "sin(t * 2) * 0.1 * m"
    }}),
    compile_track(leg_left, {{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "sin(t) * 45 * m"
    }}),
    compile_track(leg_right, {{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "-sin(t) * 45 * m"
    }}),
    compile_track(leg_left_btm, {{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "(-sin(t * 2) * 45 - 45) * m"
    }}),
    compile_track(leg_right_btm, {{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "(-sin(t * 2) * 45 - 45) * m"
    }}),
    compile_track(hand_left, {{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "-sin(t) * 45 * m"
    }}),
    compile_track(hand_right, {{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "sin(t) * 45 * m"
    }}),
    compile_track(hand_left_btm, {{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "(sin(t * 2) * 45 + 45) * m"
    }}),
    compile_track(hand_right_btm, {{
        channel = CH_ROTATE,
        axis = AX_X,
        expression = "(sin(t * 2) * 45 + 45) * m"
    }}),
}

function on_render()
    local tm = time.uptime() * 10 + tm_offset
    local speed = vec3.length(body:get_vel()) / 7.0
    local delta = time.delta()

    prev_speed = prev_speed * (1.0 - delta * 10) + speed * delta * 10
    speed = prev_speed

    local ttm = time.precise_time()
    local matrix = mat4.idt()
    for i, track in ipairs(tracks) do
        rig:set_matrix(track.bone_index, track.generator(matrix, tm, speed))
    end

    -- rig:set_matrix(body_idx, mat4.translate({0, math.sin(tm * 2) * 0.1 * speed, 0}))

    -- rig:set_matrix(leg_left, mat4.rotate({1, 0, 0}, (math.sin(tm) * 45) * speed))
    -- rig:set_matrix(leg_right, mat4.rotate({1, 0, 0}, (-math.sin(tm) * 45) * speed))

    -- rig:set_matrix(leg_left_btm, mat4.rotate({1, 0, 0}, (-math.sin(tm * 2) * 45 - 45) * speed))
    -- rig:set_matrix(leg_right_btm, mat4.rotate({1, 0, 0}, (-math.sin(tm * 2) * 45 - 45) * speed))

    -- rig:set_matrix(hand_left, mat4.rotate({1, 0, 0}, (-math.sin(tm) * 45) * speed))
    -- rig:set_matrix(hand_right, mat4.rotate({1, 0, 0}, (math.sin(tm) * 45) * speed))

    -- rig:set_matrix(hand_left_btm, mat4.rotate({1, 0, 0}, (math.sin(tm * 2) * 45 + 45) * speed))
    -- rig:set_matrix(hand_right_btm, mat4.rotate({1, 0, 0}, (math.sin(tm * 2) * 45 + 45) * speed))
    print(math.floor((time.precise_time() - ttm) * 1e6), "mcs")
end
