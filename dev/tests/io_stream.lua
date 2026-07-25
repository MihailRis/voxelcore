local device = file.create_memory_device()
file.write(device..":test.txt", "Hello\nWorld")
file.write_bytes(device..":test.bin", Bytearray({20, 30, 100, 200, 255}))

local stream = file.open(device..":test.txt", 'r')

local line1 = stream:read_line()
local line2 = stream:read_line()

assert(line1 == "Hello")
assert(line2 == "World")

local stream2 = file.open(device..":test.bin", 'rb')
local data = stream2:read(5)
assert(data[1] == 20)
assert(data[2] == 30)
assert(data[3] == 100)
assert(data[4] == 200)
assert(data[5] == 255)

local stream3 = file.open(device..":test.txt", 'r')
assert(stream3:is_alive())
stream3:close()
assert(not stream3:is_alive())

local stream4 = file.open(device..":test.txt", 'r')
stream4:seek('b', 6)
local line = stream4:read_line()
assert(line == "World")

local stream5 = file.open(device..":test.txt", 'r')
stream5:seek('e', 0)
local pos = stream5:tell()
assert(pos == 11)
