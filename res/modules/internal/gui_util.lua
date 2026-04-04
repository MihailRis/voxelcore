local gui_util = {
    local_dispatchers = {}
}

--- Parse `pagename?arg1=value1&arg2=value2` queries
--- @param query string page query string
--- @return string, string -> page_name, args_table
function gui_util.parse_query(query)
    local args = {}
    local name

    local index = string.find(query, '?')
    if index then
        local argstr = string.sub(query, index + 1)
        name = string.sub(query, 1, index - 1)
        
        for key, value in string.gmatch(argstr, "([^=&]*)=([^&]*)") do
            args[key] = value
        end
    else
        name = query
    end
    return name, args
end

--- @param query string page query string
function gui_util.load_page(query)
    local name, args = gui_util.parse_query(query)
    for i = #gui_util.local_dispatchers, 1, -1 do
        local newname, newargs = gui_util.local_dispatchers[i](name, args)
        name = newname or name
        args = newargs or args
    end
    local filename = file.find(string.format("layouts/pages/%s.xml", name))
    if filename then
        name = file.prefix(filename)..":pages/"..name
        gui.load_document(filename, name, args, { app = __vc_app })
        return name
    end
end

function gui_util.add_page_dispatcher(dispatcher)
    if type(dispatcher) ~= "function" then
        error("function expected")
    end
    table.insert(gui_util.local_dispatchers, dispatcher)
end

function gui_util.__reset_local()
    gui_util.local_dispatchers = {}
end

-- class designed for simple UI-nodes access via properties syntax
local Element = {}
local __gui_getattr = gui.getattr
local __gui_setattr = gui.setattr
function Element.new(docname, name)
    return setmetatable({docname=docname, name=name}, {
        __index=function(self, k)
            return __gui_getattr(self.docname, self.name, k)
        end,
        __newindex=function(self, k, v)
            __gui_setattr(self.docname, self.name, k, v)
        end,
        __ipairs=function(self)
            local i = 0
            return function()
                i = i + 1
                local elem = __gui_getattr(self.docname, self.name, i)
                if elem == nil then
                    return
                end
                return i, elem
            end
        end
    })
end

-- the engine automatically creates an instance for every ui document (layout)
local Document = {}
function Document.new(docname)
    return setmetatable({name=docname}, {
        __index=function(self, k)
            local elem = Element.new(self.name, k)
            rawset(self, k, elem)
            return elem
        end
    })
end

local RadioGroup = {}
function RadioGroup:set(key)
    if type(self) ~= 'table' then
        error("called as non-OOP via '.', use radiogroup:set")
    end
    if self.current then
        self.elements[self.current].enabled = true
    end
    self.elements[key].enabled = false
    self.current = key
    if self.callback then
        self.callback(key)
    end
end
function RadioGroup:__call(elements, onset, default)
    local group = setmetatable({
        elements=elements,
        callback=onset,
        current=nil
    }, {__index=self})
    group:set(default)
    return group
end
setmetatable(RadioGroup, RadioGroup)

gui_util.Document = Document
gui_util.Element = Element
gui_util.RadioGroup = RadioGroup

function gui.show_message(text, actual_callback)
    local id = "dialog_"..random.uuid()

    local callback = function()
        gui.root[id]:destruct()
        if actual_callback then
            actual_callback()
        end
    end
    gui.root.root:add(string.format([[
        <container id='%s' color='#00000080' size-func='-1,-1' z-index='10'>
            <panel color='#507090E0' size='300' padding='16'
                   gravity='center-center' interval='4'>
                <label>%s</label>
                <button onclick='DATA.callback()'>@OK</button>
            </panel>
        </container>
    ]], id, string.escape_xml(text)), {callback=callback})
    input.add_callback("key:escape", callback, gui.root[id])
end

function gui.show_input_dialog(text, actual_callback, validator, confirm_text)
    local id = "dialog_"..random.uuid()

    local callback = function()
        if not gui.root[id.."_input"].valid then
            return
        end
        gui.root[id]:destruct()
        if actual_callback then
            actual_callback(gui.root[id.."_input"].text)
        end
    end
    gui.root.root:add(string.format([[
        <container id='%s' color='#00000080' size-func='-1,-1' z-index='10'>
            <panel color='#507090E0' size='600' padding='16'
                   gravity='center-center' interval='4'>
                <label>%s</label>
                <textbox id='%s_input' validator='DATA.validator'/>
                <button onclick='DATA.callback()'>%s</button>
            </panel>
        </container>
    ]], id, string.escape_xml(text), id, string.escape_xml(confirm_text or gui.str("OK"))), {
        callback=callback,
        validator=validator or function() return #text > 0 end
    })
    input.add_callback("key:escape", function() gui.root[id]:destruct() end, gui.root[id])
    input.add_callback("key:enter", callback, gui.root[id])
    gui.root[id.."_input"].focused = true
end


function gui.ask(text, on_yes, on_no, yes_text, no_text)
    on_yes = on_yes or function() end
    on_no = on_no or function() end
    yes_text = yes_text == "" and "Yes" or (yes_text or "Yes")
    no_text = no_text or "No"

    local id = "dialog_"..random.uuid()

    local yes_callback = function()
        gui.root[id]:destruct()
        on_yes()
    end
    local no_callback = function()
        gui.root[id]:destruct()
        on_no()
    end
    gui.root.root:add(string.format([[
        <container id='%s' color='#00000080' size-func='-1,-1' z-index='10'>
            <panel color='#507090E0' size='600' padding='16'
                   gravity='center-center' interval='4'>
                <label margin='4'>%s</label>
                <button onclick='DATA.on_yes()'>@%s</button>
                <button onclick='DATA.on_no()'>@%s</button>
            </panel>
        </container>
    ]], id, string.escape_xml(text)), {on_yes=yes_callback, on_no=no_callback},
        string.escape_xml(yes_text), string.escape_xml(no_text))
    input.add_callback("key:escape", no_callback, gui.root[id])
end

return gui_util
