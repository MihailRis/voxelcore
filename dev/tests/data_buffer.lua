local data_buffer = require "core:data_buffer"
local buffer = data_buffer({}, 'LE', true)
buffer:put_int16(2025)
debug.print(buffer)
