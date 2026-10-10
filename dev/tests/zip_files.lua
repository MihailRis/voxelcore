local function sorted_list(path)
    local entries = file.list(path)
    table.sort(entries)
    return entries
end

local function check_list(expected, path)
    local entries = sorted_list(path)
    asserts.equals(#expected, #entries)
    for i, name in ipairs(expected) do
        asserts.equals(name, entries[i])
    end
end

local function join(root, name)
    if root:sub(-1) == ":" then
        return root..name
    end
    return root.."/"..name
end

local bytes = {0xDE, 0xAD, 0x00, 0xC0, 0xDE}

local function create_tree(root)
    file.mkdirs(join(root, "sub/deep"))
    file.mkdir(join(root, "empty"))
    file.write(join(root, "root.txt"), "root file")
    file.write(join(root, "sub/a.txt"), "example, пример")
    file.write_bytes(join(root, "sub/deep/binary"), bytes)
end

local function check_tree(m)
    debug.log("check directories")
    assert(file.exists(m..":"))
    assert(file.isdir(m..":"))
    assert(file.isdir(m..":sub"))
    assert(file.isdir(m..":sub/deep"))
    assert(file.isdir(m..":empty"))
    assert(not file.isfile(m..":sub"))

    debug.log("check files")
    assert(file.isfile(m..":root.txt"))
    assert(file.isfile(m..":sub/a.txt"))
    assert(not file.isdir(m..":sub/a.txt"))
    assert(not file.exists(m..":missing.txt"))
    assert(not file.exists(m..":sub/missing.txt"))

    debug.log("read files")
    asserts.equals("root file", file.read(m..":root.txt"))
    asserts.equals("example, пример", file.read(m..":sub/a.txt"))
    asserts.equals(#"root file", file.length(m..":root.txt"))

    local rbytes = file.read_bytes(m..":sub/deep/binary")
    asserts.equals(#bytes, #rbytes)
    for i, b in ipairs(bytes) do
        asserts.equals(b, rbytes[i])
    end

    debug.log("read file with io_stream")
    local stream = file.open(m..":root.txt", "r")
    asserts.equals("root file", stream:read_line())
    stream:close()

    debug.log("list directories")
    check_list({m..":empty", m..":root.txt", m..":sub"}, m..":")
    check_list({m..":sub/a.txt", m..":sub/deep"}, m..":sub")
    check_list({m..":sub/deep/binary"}, m..":sub/deep")
    check_list({}, m..":empty")

    debug.log("check read-only")
    assert(not file.is_writeable(m..":"))
    assert(not pcall(file.write, m..":new.txt", "text"))
    local ok, created = pcall(file.mkdir, m..":newdir")
    assert(not (ok and created))
    assert(not file.exists(m..":newdir"))
end

local function check_zip(src, zipfile)
    debug.log("create zip "..zipfile.." from "..src)
    file.create_zip(src, zipfile)
    assert(file.isfile(zipfile))

    debug.log("mount "..zipfile)
    local m = file.mount(zipfile)
    check_tree(m)

    debug.log("unmount "..m)
    file.unmount(m)
    assert(not pcall(file.read, m..":root.txt"))
    assert(not pcall(file.unmount, m))
end

local mem = file.create_memory_device()
local out = file.create_memory_device()

debug.log("zip memory device root")
create_tree(mem..":")
check_zip(mem..":", out..":root.zip")

debug.log("zip memory device subdirectory")
create_tree(mem..":pack")
check_zip(mem..":pack", out..":pack.zip")

debug.log("zip nested subdirectory")
create_tree(mem..":a/b/pack")
check_zip(mem..":a/b/pack", out..":nested.zip")

debug.log("zip real filesystem directory")
if file.exists("config:ziptest") then
    file.remove_tree("config:ziptest")
end
create_tree("config:ziptest/pack")
check_zip("config:ziptest/pack", "config:ziptest/pack.zip")
file.remove_tree("config:ziptest")

debug.log("mount non-zip file")
file.write(out..":text.txt", "not an archive")
assert(not pcall(file.mount, out..":text.txt"))
