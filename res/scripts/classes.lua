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

local ServerSocket = {__index={
    close=function(self) return network.__closeserver(self.id) end,
    is_open=function(self) return network.__is_serveropen(self.id) end,
    get_port=function(self) return network.__get_serverport(self.id) end,
}}


local _tcp_server_callbacks = {}
local _tcp_client_callbacks = {}

network.tcp_open = function (port, handler)
    local socket = setmetatable({id=network.__open(port)}, ServerSocket)

    _tcp_server_callbacks[socket.id] = function(id)
        handler(setmetatable({id=id}, Socket))
    end
    return socket
end

network.tcp_connect = function(address, port, callback)
    local socket = setmetatable({id=0}, Socket)
    socket.id = network.__connect(address, port)
    _tcp_client_callbacks[socket.id] = function() callback(socket) end
    return socket
end

network.__process_events = function()
    local CLIENT_CONNECTED = 1
    local CONNECTED_TO_SERVER = 2

    local cleaned = false
    local events = network.__pull_events()
    for i, event in ipairs(events) do
        local etype, sid, cid = unpack(event)

        if etype == CLIENT_CONNECTED then
            local callback = _tcp_server_callbacks[sid]
            if callback then
                callback(cid)
            end
        elseif etype == CONNECTED_TO_SERVER then
            local callback = _tcp_client_callbacks[cid]
            if callback then
                callback()
            end
        end

        -- remove dead servers
        if not cleaned then
            for sid, _ in pairs(_tcp_server_callbacks) do
                if not network.__is_serveropen(sid) then
                    _tcp_server_callbacks[sid] = nil
                end
            end
            for cid, _ in pairs(_tcp_client_callbacks) do
                if not network.__is_alive(cid) then
                    _tcp_client_callbacks[cid] = nil
                end
            end
            cleaned = true
        end
    end
end
