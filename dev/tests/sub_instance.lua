app.create_memory_device("memtest")

local script_path = "config:subprocess.lua"
local log_path = "config:subprocess.log"

file.write(script_path, [[
    debug.log("hello from sub-instance")
    sleep(1)
]])

local handle = app.start_background_instance(script_path, log_path)
assert(handle >= 0)

local stdin = app.get_stdin_stream(handle)
assert(stdin ~= nil)
stdin:write_line("sub_instance_test")
stdin:flush()

app.sleep_until(function() return not app.is_instance_alive(handle) end)
assert(not app.is_instance_alive(handle))

local stdout = app.get_stdout_stream(handle)
assert(stdout ~= nil)
local output = table.concat(table.map(stdout:read(1e6), function(_, line) return "<sub>"..line end), '\n')
print(output)
assert(output and output:find("hello from sub-instance", 1, true) ~= nil)
assert(output and output:find('stdin-cmd] unknown command "sub_instance_test"', 1, true) ~= nil)
stdout:close()

file.remove(script_path)
assert(not app.terminate_instance(handle))
