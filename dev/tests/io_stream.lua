local device = file.create_memory_device()

local function path(name) return device .. ":" .. name end

local function eq_list(actual, expected, msg)
    asserts.equals(#actual, #expected, (msg or "list") .. " length")
    for i = 1, #expected do
        asserts.equals(actual[i], expected[i], (msg or "list") .. "[" .. i .. "]")
    end
end

file.write(path("test.txt"), "Hello\nWorld")
file.write_bytes(path("test.bin"), Bytearray({20, 30, 100, 200, 255}))

do
    local stream = file.open(path("test.txt"), 'r')
    asserts.equals(stream:read_line(), "Hello")
    asserts.equals(stream:read_line(), "World")
    stream:close()
end

do
    local stream = file.open(path("test.bin"), 'rb')
    local data = stream:read(5)
    asserts.equals(data[1], 20); asserts.equals(data[2], 30); asserts.equals(data[3], 100); asserts.equals(data[4], 200); asserts
    .equals(data[5], 255)
    stream:close()
end

do
    local stream = file.open(path("test.txt"), 'r')
    assert(stream:is_alive())
    assert(not stream:is_closed())
    stream:close()
    assert(not stream:is_alive())
    assert(stream:is_closed())
end

do
    local stream = file.open(path("test.txt"), 'r')
    stream:seek('b', 6)
    asserts.equals(stream:read_line(), "World")
    stream:close()
end

do
    local stream = file.open(path("test.txt"), 'r')
    stream:seek('e', 0)
    asserts.equals(stream:tell(), 11)
    stream:close()
end

-------------------------------------------------------------------------------
-- text: line endings and end of stream
-------------------------------------------------------------------------------

do
    local stream = file.open(path("test.txt"), 'r')
    stream:read_line(); stream:read_line()
    asserts.equals(stream:read_line(), nil, "after last line")
    asserts.equals(stream:read_line(), nil, "after last line, again")
    stream:close()
end

do
    file.write(path("empty.txt"), "")
    local stream = file.open(path("empty.txt"), 'r')
    asserts.equals(stream:read_line(), nil, "empty file")
    stream:close()
end

do
    file.write(path("crlf.txt"), "a\r\nb\rc\nd\r")
    local stream = file.open(path("crlf.txt"), 'r')
    asserts.equals(stream:read_line(), "a", "CRLF")
    asserts.equals(stream:read_line(), "b\rc", "lone CR is kept")
    asserts.equals(stream:read_line(), "d\r", "CR at end of stream")
    asserts.equals(stream:read_line(), nil)
    stream:close()
end

-------------------------------------------------------------------------------
-- text: read(count) and read_fully
-------------------------------------------------------------------------------

do
    file.write(path("blank.txt"), "\n\nA\nB\n\n")
    local stream = file.open(path("blank.txt"), 'r')
    eq_list(stream:read(10), {"A", "B"}, "trimmed")
    stream:close()
end

do
    local stream = file.open(path("blank.txt"), 'r')
    eq_list(stream:read(10, false), {"", "", "A", "B", ""}, "untrimmed")
    stream:close()
end

do
    file.write(path("only_blank.txt"), "\n\n\n")
    local stream = file.open(path("only_blank.txt"), 'r')
    eq_list(stream:read(5), {}, "only empty lines")
    stream:close()
end

do
    local stream = file.open(path("test.txt"), 'r')
    eq_list(stream:read(5), {"Hello", "World"}, "past end of stream")
    stream:close()
end

do
    local stream = file.open(path("test.txt"), 'r')
    assert(not pcall(stream.read, stream, -1), "negative count must fail")
    stream:close()
end

do
    local stream = file.open(path("test.txt"), 'r')
    asserts.equals(stream:read(), "Hello")
    stream:close()
end

do
    local stream = file.open(path("test.txt"), 'r')
    asserts.equals(stream:read_fully(), "Hello\nWorld", "read_fully as string")
    stream:close()
end

do
    local stream = file.open(path("test.txt"), 'r')
    eq_list(stream:read_fully(true), {"Hello", "World"}, "read_fully as lines")
    stream:close()
end

-------------------------------------------------------------------------------
-- text: writing
-------------------------------------------------------------------------------

do
    local stream = file.open(path("out.txt"), 'w')
    stream:write("one")
    stream:write({"two", "three"})
    stream:write_line("four")
    stream:close()
    asserts.equals(file.read(path("out.txt")), "one\ntwo\nthree\nfour\n", "text write")
end

do
    local stream = file.open(path("out.txt"), 'w')
    assert(not pcall(stream.write, stream, 42), "writing a number in text mode must fail")
    stream:close()
end

-------------------------------------------------------------------------------
-- binary
-------------------------------------------------------------------------------

do
    local stream = file.open(path("test.bin"), 'rb')
    eq_list(stream:read(5, true), {20, 30, 100, 200, 255}, "read as table")
    stream:close()
end

do
    local stream = file.open(path("test.bin"), 'rb')
    stream:read(3)
    asserts.equals(#stream:read(10), 2, "short read")
    asserts.equals(#stream:read(10), 0, "read at end of stream")
    stream:close()
end

do
    local stream = file.open(path("test.bin"), 'rb')
    local data = stream:read_fully()
    asserts.equals(#data, 5); asserts.equals(data[1], 20); asserts.equals(data[5], 255)
    stream:close()
end

do
    local stream = file.open(path("test.bin"), 'rb')
    eq_list(stream:read_fully(true), {20, 30, 100, 200, 255}, "read_fully as table")
    stream:close()
end

do
    local stream = file.open(path("out.bin"), 'wb')
    stream:write({1, 2, 3})
    stream:write(Bytearray({4, 5}))
    stream:close()

    local data = file.read_bytes(path("out.bin"))
    asserts.equals(#data, 5, "binary write length")
    for i = 1, 5 do asserts.equals(data[i], i, "binary write byte " .. i) end
end

do
    local stream = file.open(path("fmt.bin"), 'wb')
    stream:write("<iH", -5, 1000)
    stream:close()

    stream = file.open(path("fmt.bin"), 'rb')
    local a, b = stream:read("<iH")
    asserts.equals(a, -5, "packed int")
    asserts.equals(b, 1000, "packed ushort")
    stream:close()
end

do
    local stream = file.open(path("test.bin"), 'rb')
    assert(not pcall(stream.read, stream), "binary read without args must fail")
    assert(not pcall(stream.read, stream, true), "binary read with boolean must fail")
    stream:close()
end

-------------------------------------------------------------------------------
-- settings
-------------------------------------------------------------------------------

do
    local stream = file.open(path("test.txt"), 'r')
    stream:set_binary_mode(true)
    assert(stream:is_binary_mode())
    stream:set_binary_mode(false)
    assert(not stream:is_binary_mode(), "set_binary_mode(false)")
    stream:close()
end

do
    local stream = file.open(path("test.txt"), 'rb')
    asserts.equals(stream:read_line(), "Hello")
    stream:close()
end

do
    local stream = file.open(path("test.txt"), 'r')
    asserts.equals(stream:get_mode(), "default")
    asserts.equals(stream:get_flush_mode(), "all")
    assert(not pcall(stream.set_mode, stream, "nope"), "invalid mode must fail")
    assert(not pcall(stream.set_flush_mode, stream, "nope"), "invalid flush mode must fail")
    asserts.equals(stream:get_mode(), "default", "mode unchanged after error")
    stream:close()
end

-------------------------------------------------------------------------------
-- buffered mode
-------------------------------------------------------------------------------

do
    local stream = file.open(path("test.txt"), 'r')
    stream:set_mode("buffered")
    asserts.equals(stream:read_line(), "Hello")
    asserts.equals(stream:read_line(), "World")
    asserts.equals(stream:read_line(), nil)
    stream:close()
end

do
    local stream = file.open(path("test.txt"), 'r')
    stream:set_mode("buffered")
    stream:read_line()
    asserts.equals(stream:available(), 5, "available")
    assert(stream:available(5))
    assert(not stream:available(6))
    asserts.equals(stream:tell(), 6, "tell after buffered read")
    stream:close()
end

do
    local stream = file.open(path("test.txt"), 'r')
    stream:set_mode("buffered")
    stream:read_line()
    stream:seek('c', 1)
    asserts.equals(stream:read_line(), "orld", "seek 'c' in buffered mode")
    stream:close()
end

do
    local stream = file.open(path("test.txt"), 'r')
    stream:set_mode("buffered")
    stream:read_line()
    stream:seek('b', 0)
    asserts.equals(stream:read_line(), "Hello", "seek 'b' in buffered mode")
    stream:close()
end

do
    local stream = file.open(path("test.txt"), 'r')
    stream:set_mode("buffered")
    stream:set_max_buffer_size(2)
    asserts.equals(stream:get_max_buffer_size(), 2)
    eq_list(stream:read_fully(true), {"Hello", "World"}, "tiny buffer")
    stream:close()
end

do
    local stream = file.open(path("buf.bin"), 'wb')
    stream:set_mode("buffered")
    stream:write({1, 2, 3})
    asserts.equals(stream:tell(), 3, "tell with pending writes")
    stream:close()
end

do
    local stream = file.open(path("buf.txt"), 'w')
    stream:set_mode("buffered")
    stream:write("abc")
    stream:close()
    asserts.equals(file.read(path("buf.txt")), "abc\n", "close flushes")
end

do
    local stream = file.open(path("buf.txt"), 'w')
    stream:set_mode("buffered")
    stream:write("abc")
    stream:set_mode("default")
    stream:write("def")
    stream:close()
    asserts.equals(file.read(path("buf.txt")), "abc\ndef\n", "set_mode flushes")
end

do
    local stream = file.open(path("big.bin"), 'wb')
    stream:set_mode("buffered")
    stream:set_max_buffer_size(4)

    stream:write({1, 2})
    stream:write({3, 4, 5, 6, 7, 8, 9, 10, 11, 12})
    stream:write({13})
    stream:close()

    local data = file.read_bytes(path("big.bin"))
    asserts.equals(#data, 13, "big write length")
    for i = 1, 13 do asserts.equals(data[i], i, "big write byte " .. i) end
end

do
    for _, flush_mode in ipairs({"all", "buffer"}) do
        local stream = file.open(path("flush.txt"), 'w')
        stream:set_mode("buffered")
        stream:set_flush_mode(flush_mode)
        stream:write("x")
        stream:flush()
        asserts.equals(file.read(path("flush.txt")), "x\n", "flush mode " .. flush_mode)
        stream:close()
    end
end

-------------------------------------------------------------------------------
-- yield mode
-------------------------------------------------------------------------------

do
    local stream = file.open(path("test.txt"), 'r')
    stream:set_mode("yield")

    local co = coroutine.create(function()
        return stream:read_line(), stream:read_fully()
    end)

    local ok, line, rest = coroutine.resume(co)
    assert(ok, line)
    asserts.equals(coroutine.status(co), "dead", "yield mode read_fully must not hang")
    asserts.equals(line, "Hello")
    asserts.equals(rest, "World")
    stream:close()
end

do
    local stream = file.open(path("test.bin"), 'rb')
    stream:set_mode("yield")

    local co = coroutine.create(function() return stream:read(100) end)

    local ok, err = coroutine.resume(co)
    assert(ok, err)
    asserts.equals(coroutine.status(co), "suspended", "short read must yield")

    stream:close()

    local ok2, data = coroutine.resume(co)
    assert(ok2, data)
    asserts.equals(coroutine.status(co), "dead", "closed stream ends the wait")
    asserts.equals(#data, 5, "bytes read before close")
end
