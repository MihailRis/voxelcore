debug.log("check initial state")
assert(file.exists("config:"))

debug.log("write text file")
assert(file.write("config:text.txt", "example, пример"))
assert(file.exists("config:text.txt"))
assert(file.isfile("config:text.txt"))

debug.log("read text file")
assert(file.read("config:text.txt") == "example, пример")

debug.log("delete file")
file.remove("config:text.txt")
assert(not file.exists("config:text.txt"))

debug.log("create directory")
file.mkdir("config:dir")
assert(file.isdir("config:dir"))

debug.log("remove directory")
file.remove("config:dir")
assert(not file.isdir("config:dir"))

debug.log("create directories")
file.mkdirs("config:dir/subdir/other")
assert(file.isdir("config:dir/subdir/other"))

debug.log("list directory")
file.write("config:dir/subdir/a.txt", "helloworld")
file.write("config:dir/subdir/b.txt", "gfgsfhs")

local entries = file.list("config:dir/subdir")
assert(#entries == 3)
table.sort(entries)
assert(entries[1] == "config:dir/subdir/a.txt")
assert(entries[2] == "config:dir/subdir/b.txt")
assert(entries[3] == "config:dir/subdir/other")

debug.log("remove tree")
file.remove_tree("config:dir")
assert(not file.isdir("config:dir"))

debug.log("write binary file")
local bytes = {0xDE, 0xAD, 0xC0, 0xDE}
file.write_bytes("config:binary", bytes)
assert(file.exists("config:binary"))

debug.log("read binary file")
local rbytes = file.read_bytes("config:binary")
assert(#rbytes == #bytes)
for i, b in ipairs(bytes) do
    assert(rbytes[i] == b)
end

debug.log("delete file")
file.remove("config:binary")
assert(not file.exists("config:binary"))

debug.log("checking entry points for writeability")
assert(file.is_writeable("config:"))
assert(file.is_writeable("export:"))
assert(not file.is_writeable("user:"))
assert(not file.is_writeable("res:"))
