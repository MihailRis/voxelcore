app.create_memory_device("memtest")

local script_path = "config:subprocess.lua"
local log_path = "config:subprocess.log"

file.write(script_path, [[
    debug.log("hello from sub-instance")
]])

local handle = app.start_background_instance(script_path, log_path)
assert(handle >= 0)

app.sleep_until(function() return not app.is_instance_alive(handle) end)
assert(not app.is_instance_alive(handle))

local stdout = app.get_stdout_stream(handle)
assert(stdout ~= nil)
local output = table.concat(stdout:read(1e6), '\n')
assert(output and output:find("hello from sub-instance", 1, true) ~= nil)
stdout:close()

file.remove(script_path)
assert(not app.terminate_instance(handle))
