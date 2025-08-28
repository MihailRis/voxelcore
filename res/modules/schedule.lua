local Schedule = {
    __index = {
        set_interval = function(self, ms, callback, repetions)
            local id = self._next_interval
            self._intervals[id] = {
                last_called = 0.0,
                delay = ms / 1000.0,
                callback = callback,
                repetions = repetions,
            }
            self._next_interval = id + 1
            return id
        end,
        tick = function(self, dt)
            local timer = self._timer + dt
            for id, interval in pairs(self._intervals) do
                if timer - interval.last_called >= interval.delay then
                    local stack_size = debug.count_frames()
                    xpcall(interval.callback, function(msg)
                        __vc__error(msg, 1, 1, stack_size)
                    end)
                    interval.last_called = timer
                    local repetions = interval.repetions
                    if repetions then
                        if repetions <= 1 then
                            self:remove_interval(id)
                        else
                            interval.repetions = repetions - 1
                        end
                    end
                end
            end
            self._timer = timer
        end,
        remove_interval = function (self, id)
           self._intervals[id] = nil
        end
    }
}

return function ()
    return setmetatable({
        _next_interval = 1,
        _timer = 0.0,
        _intervals = {},
    }, Schedule)
end
