local __app = __vc_app

local function get_node_center(node)
    local pos = node.wpos
    local size = node.size
    return pos[1] + size[1] / 2, pos[2] + size[2] / 2
end

local function types_check_enum(value, enum)
    if value == nil then
        return true, nil
    end
    if type(value) ~= "string" then
        return false, string.format("string expected [%s]", table.concat(table.keys(enum), "|"))
    end
    return true, nil
end

local function types_check_number(value)
    if value == nil then
        return true, nil
    end
    local value_type = type(value)
    if value_type ~= "number" then
        return false, string.format("number expected, got %s", value_type)
    end
end

local function check_option(t, key, func, ...)
    local value = t[key]
    if value == nil then
        return nil
    end
    local success, message = func(value, ...)
    if not success then
        error(string.format("invalid option '%s': %s", key, message))
    end
end

local buttons_enum = {
    left = 0,
    right = 1,
    middle = 2,
}
local no_options = {}

local _tick = __app.tick
local _set_button_pressed = test.set_button_pressed
local _set_key_pressed = test.set_key_pressed
local _enter_text = test.enter_text
test.set_button_pressed = nil
test.set_key_pressed = nil
test.enter_text = nil

function test.click(node, options)
    options = options or no_options

    check_option(options, 'button', types_check_enum, buttons_enum)
    check_option(options, 'timeout', types_check_number)

    local button = buttons_enum[options.button or 'left']
    local timeout = options.timeout or 1000

    local tm = time.precise_time()
    while (time.precise_time() - tm) * 1000 < timeout do
        if node.exists then
            break
        end
        _tick()
    end
    if not node.exists then
        error(string.format("timeout %s ms exceeded: element %s does not exists",
            math.floor(timeout), string.escape(rawget(node, 'name'))))
    end

    local center_x, center_y = get_node_center(node)
    _set_button_pressed(button, center_x, center_y, true)
    _tick()
    _set_button_pressed(button, center_x, center_y, false)
    _tick()
end

-- TODO: replace with resolving in C++ side
function test.find_by_attr(node, attr, value)
    if getmetatable(node) == Document then
        return test.find_by_attr(node.root, attr, value)
    end
    if node[attr] == value then
        return node
    end
    local i = 1
    while true do
        local subnode = node[i]
        if subnode == nil then
            break
        end
        local found = test.find_by_attr(subnode, attr, value)
        if found then
            return found
        end
        i = i + 1
    end
end

function test.find_by_text(node, text)
    return test.find_by_attr(node, "text", text)
end

function test.find_by_attr_presence(node, attr)
    if node[attr] then
        return node
    end
    local i = 1
    while true do
        local subnode = node[i]
        if subnode == nil then
            break
        end
        local found = test.find_by_attr_presence(subnode, attr)
        if found then
            return found
        end
        i = i + 1
    end
end

function test.fill(node, text, options)
    test.click(node, options)
    __app.tick()
    _enter_text(text)
    test.press("enter")
end

function test.press(key)
    _set_key_pressed(key, true)
    __app.tick()
    _set_key_pressed(key, false)
    __app.tick()
end
