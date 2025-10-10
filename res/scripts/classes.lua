local Camera = {__index={
    get_pos=function(self) return cameras.get_pos(self.cid) end,
    set_pos=function(self, v) return cameras.set_pos(self.cid, v) end,
    get_name=function(self) return cameras.name(self.cid) end,
    get_index=function(self) return self.cid end,
    get_rot=function(self) return cameras.get_rot(self.cid) end,
    set_rot=function(self, m) return cameras.set_rot(self.cid, m) end,
    get_zoom=function(self) return cameras.get_zoom(self.cid) end,
    set_zoom=function(self, f) return cameras.set_zoom(self.cid, f) end,
    get_fov=function(self) return cameras.get_fov(self.cid) end,
    set_fov=function(self, f) return cameras.set_fov(self.cid, f) end,
    is_perspective=function(self) return cameras.is_perspective(self.cid) end,
    set_perspective=function(self, b) return cameras.set_perspective(self.cid, b) end,
    is_flipped=function(self) return cameras.is_flipped(self.cid) end,
    set_flipped=function(self, b) return cameras.set_flipped(self.cid, b) end,
    get_front=function(self) return cameras.get_front(self.cid) end,
    get_right=function(self) return cameras.get_right(self.cid) end,
    get_up=function(self) return cameras.get_up(self.cid) end,
    look_at=function(self, v, f) return cameras.look_at(self.cid, v, f) end,
}}

local wrappers = {}

cameras.get = function(name)
    if type(name) == 'number' then
        return cameras.get(cameras.name(name))
    end
    local wrapper = wrappers[name]
    if wrapper ~= nil then
        return wrapper
    end
    local cid = cameras.index(name)
    wrapper = setmetatable({cid=cid}, Camera)
    wrappers[name] = wrapper
    return wrapper
end


local Socket = {__index={
    send=function(self, ...) return network.__send(self.id, ...) end,
    recv=function(self, ...) return network.__recv(self.id, ...) end,
    close=function(self) return network.__close(self.id) end,
    available=function(self) return network.__available(self.id) or 0 end,
    is_alive=function(self) return network.__is_alive(self.id) end,
    is_connected=function(self) return network.__is_connected(self.id) end,
    get_address=function(self) return network.__get_address(self.id) end,
}}

local WriteableSocket = {__index={
    send=function(self, ...) return network.__send(self.id, ...) end,
    close=function(self) return network.__close(self.id) end,
    is_open=function(self) return network.__is_alive(self.id) end,
    get_address=function(self) return network.__get_address(self.id) end,
}}

local ServerSocket = {__index={
    close=function(self) return network.__closeserver(self.id) end,
    is_open=function(self) return network.__is_serveropen(self.id) end,
    get_port=function(self) return network.__get_serverport(self.id) end,
}}

local DatagramServerSocket = {__index={
    close=function(self) return network.__closeserver(self.id) end,
    is_open=function(self) return network.__is_serveropen(self.id) end,
    get_port=function(self) return network.__get_serverport(self.id) end,
    send=function(self, ...) return network.__udp_server_send_to(self.id, ...) end
}}

local _tcp_server_callbacks = {}
local _tcp_client_callbacks = {}

local _udp_server_callbacks = {}
local _udp_client_datagram_callbacks = {}
local _udp_client_open_callbacks = {}
local _http_response_callbacks = {}
local _http_error_callbacks = {}

network.get = function(url, callback, errorCallback, headers)
    local id = network.__get(url, headers)
    if callback then
        _http_response_callbacks[id] = callback
    end
    if errorCallback then
        _http_error_callbacks[id] = errorCallback
    end
end

network.get_binary = function(url, callback, errorCallback, headers)
    local id = network.__get_binary(url, headers)
    if callback then
        _http_response_callbacks[id] = callback
    end
    if errorCallback then
        _http_error_callbacks[id] = errorCallback
    end
end

network.post = function(url, data, callback, errorCallback, headers)
    local id = network.__post(url, data, headers)
    if callback then
        _http_response_callbacks[id] = callback
    end
    if errorCallback then
        _http_error_callbacks[id] = errorCallback
    end
end

network.tcp_open = function (port, handler)
    local socket = setmetatable({id=network.__open_tcp(port)}, ServerSocket)

    _tcp_server_callbacks[socket.id] = function(id)
        handler(setmetatable({id=id}, Socket))
    end
    return socket
end

network.tcp_connect = function(address, port, callback)
    local socket = setmetatable({id=0}, Socket)
    socket.id = network.__connect_tcp(address, port)
    _tcp_client_callbacks[socket.id] = function() callback(socket) end
    return socket
end

network.udp_open = function (port, datagramHandler)
    if type(datagramHandler) ~= 'function' then
        error "udp server cannot be opened without datagram handler"
    end

    local socket = setmetatable({id=network.__open_udp(port)}, DatagramServerSocket)

    _udp_server_callbacks[socket.id] = function(address, port, data)
        datagramHandler(address, port, data, socket)
    end

    return socket
end

network.udp_connect = function (address, port, datagramHandler, openCallback)
    if type(datagramHandler) ~= 'function' then
        error "udp client socket cannot be opened without datagram handler"
    end

    local socket = setmetatable({id=0}, WriteableSocket)
    socket.id = network.__connect_udp(address, port)

    _udp_client_datagram_callbacks[socket.id] = datagramHandler
    if openCallback then
        _udp_client_open_callbacks[socket.id] = function()
            openCallback(socket)
        end
    end

    return socket
end

local function clean(iterable, checkFun, ...)
    local tables = { ... }

    for id, _ in pairs(iterable) do
        if not checkFun(id) then
            for i = 1, #tables do
                tables[i][id] = nil
            end
        end
    end
end

local updating_blocks = {}
local TYPE_REGISTER = 0
local TYPE_UNREGISTER = 1

block.__perform_ticks = function(delta)
    for id, entry in pairs(updating_blocks) do
        entry.timer = entry.timer + delta
        local steps = math.floor(entry.timer / entry.delta * #entry / 3)
        if steps == 0 then
            goto continue
        end
        entry.timer = 0.0
        local event = entry.event
        local tps = entry.tps
        for i=1, steps do
            local x = entry[entry.pointer + 1]
            local y = entry[entry.pointer + 2]
            local z = entry[entry.pointer + 3]
            entry.pointer = (entry.pointer + 3) % #entry
            events.emit(event, x, y, z, tps)
        end
        ::continue::
    end
end

block.__process_register_events = function()
    local register_events = block.__pull_register_events()
    if not register_events then
        return
    end
    for i=1, #register_events, 4 do
        local header = register_events[i]
        local type = bit.band(header, 0xFFFF)
        local id = bit.rshift(header, 16)
        local x = register_events[i + 1]
        local y = register_events[i + 2]
        local z = register_events[i + 3]

        local list = updating_blocks[id]
        if type == TYPE_REGISTER then
            if not list then
                list = {}
                list.event = block.name(id) .. ".blocktick"
                list.tps = 20 / (block.properties[id]["tick-interval"] or 1)
                list.delta = 1.0 / list.tps
                list.timer = 0.0
                list.pointer = 0
                updating_blocks[id] = list
            end
            table.insert(list, x)
            table.insert(list, y)
            table.insert(list, z)
        elseif type == TYPE_UNREGISTER then
            if list then
                for j=1, #list, 3 do
                    if list[j] == x and list[j + 1] == y and list[j + 2] == z then
                        for k=1,3 do
                            table.remove(list, j)
                        end
                        j = j - 3
                    end
                end
            end
        end
    end
end

network.__process_events = function()
    local CLIENT_CONNECTED = 1
    local CONNECTED_TO_SERVER = 2
    local DATAGRAM = 3
    local RESPONSE = 4

    local ON_SERVER = 1
    local ON_CLIENT = 2

    local cleaned = false
    local events = network.__pull_events()
    for i, event in ipairs(events) do
        local etype, sid, cid, addr, port, side, data = unpack(event)

        if etype == CLIENT_CONNECTED then
            local callback = _tcp_server_callbacks[sid]
            if callback then
                callback(cid)
            end
        elseif etype == CONNECTED_TO_SERVER then
            local callback = _tcp_client_callbacks[cid] or _udp_client_open_callbacks[cid]
            if callback then
                callback()
            end
        elseif etype == DATAGRAM then
            if side == ON_CLIENT then
                local callback = _udp_client_datagram_callbacks[cid]
                if callback then
                    callback(data)
                end
            elseif side == ON_SERVER then
                local callback = _udp_server_callbacks[sid]
                if callback then
                    callback(addr, port, data)
                end
            end
        elseif etype == RESPONSE then
            if event[2] / 100 == 2 then
                local callback = _http_response_callbacks[event[3]]
                _http_response_callbacks[event[3]] = nil
                _http_error_callbacks[event[3]] = nil
                if callback then
                    callback(event[4])
                end
            else
                local callback = _http_error_callbacks[event[3]]
                _http_response_callbacks[event[3]] = nil
                _http_error_callbacks[event[3]] = nil
                if callback then
                    callback(event[2], event[4])
                end
            end
        end

        -- remove dead servers
        if not cleaned then
            clean(_tcp_server_callbacks, network.__is_serveropen, _tcp_server_callbacks)
            clean(_tcp_client_callbacks, network.__is_alive, _tcp_client_callbacks)

            clean(_udp_server_callbacks, network.__is_serveropen, _udp_server_callbacks)
            clean(_udp_client_datagram_callbacks, network.__is_alive, _udp_client_open_callbacks, _udp_client_datagram_callbacks)

            cleaned = true
        end
    end
end
