function on_open()
    document.st_blue.text = "70"
    document.st_red.text = "35"
    document.st_green.text = "90"
    document.bg_none.text = "70"
    document.bg_white.text = "35"
    document.bg_blue.text = "90"

    local t = 0.0
    document.root:setInterval(16, function()
        t = t + time.delta()
        local v = (math.sin(t * 2.0) + 1.0) / 2.0 * 100.0
        document.dyn_off.value = v
        document.dyn_slow.value = v
        document.dyn_fast.value = v
        document.dyn_off.text = string.format("%.0f", v)
        document.dyn_slow.text = string.format("%.0f", v)
        document.dyn_fast.text = string.format("%.0f", v)
    end)
end
