local gui_util = {
    local_dispatchers = {}
}

--- Parse `pagename?arg1=value1&arg2=value2` queries
--- @param query page query string
--- @return page_name, args_table
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

--- @param query page query string
--- @return document_id
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
        gui.load_document(filename, name, args)
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
function Element.new(docname, name)
    return setmetatable({docname=docname, name=name}, {
        __index=function(self, k)
            return gui.getattr(self.docname, self.name, k)
        end,
        __newindex=function(self, k, v)
            gui.setattr(self.docname, self.name, k, v)
        end,
        __ipairs=function(self)
            local i = 0
            return function()
                i = i + 1
                local elem = gui.getattr(self.docname, self.name, i)
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

return gui_util
