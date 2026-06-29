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

local env = {
    mat4 = mat4,
    X = {1, 0, 0},
    Y = {0, 1, 0},
    Z = {0, 0, 1},
    blank = core.blank,
    DST = mat4.idt(),
}
table.extend(env, math)

local function codegen_track(lines, memoised)
    local code = ""
    local translation = {false, false, false}
    local rotation = {false, false, false}
    for i, line in ipairs(lines) do
        code = code .. "\n  local l" .. i .. " = (" .. process_expression(line.expression, memoised) .. ")"
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

    for i, lineset in ipairs(linesets) do
        local lineset_code = codegen_track(lineset.lines, memoised)
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
    local generator, err = load(src, "<expr>", "bt", env)
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
}

local track = compile_track(linesets)

function on_render()
    local tm = time.uptime() * 10 + tm_offset
    local speed = vec3.length(body:get_vel()) / 7.0
    local delta = time.delta()

    prev_speed = prev_speed * (1.0 - delta * 10) + speed * delta * 10
    speed = prev_speed

    local ttm = time.precise_time()
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
    print(math.floor((time.precise_time() - ttm) * 1e6), "mcs")
end
