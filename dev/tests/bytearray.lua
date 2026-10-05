local arr = Bytearray()
assert(#arr == 0)

for i=1,10 do
    arr[i] = 10 - i
    assert(#arr == i)
    assert(arr[i] == 10 - i)
end

for i, v in ipairs(arr) do
    assert(v == 10 - i)
end

Bytearray.remove(arr, 2)
assert(#arr == 9)

Bytearray.insert(arr, {5, 3, 6})

assert(#arr == 12)
Bytearray.insert(arr, 2, 8)
asserts.equals(13, #arr)
for i=1,10 do
    assert(arr[i] == 10 - i)
end
print(#arr, arr:get_capacity())
arr:trim()
asserts.equals(#arr, arr:get_capacity())

arr = Bytearray({0, 2, 7, 1, 16, 75, 25})
assert(arr[6] == 75)

arr:insert(2, {5, 6})

asserts.equals(25, arr[#arr])
asserts.equals(5, arr[2])
asserts.equals(6, arr[3])

-- =============================

arr = Bytearray({254, 255, 255, 255, 255, 255, 255, 255})
asserts.equals(U64view(arr)[1], ctypes.uint64(2) ^ 64 - 2)
asserts.equals(I64view(arr)[1], -2)
asserts.equals(U32view(arr)[1], 2 ^ 32 - 2)
asserts.equals(I32view(arr)[1], -2)
asserts.equals(U16view(arr)[1], 2 ^ 16 - 2)
asserts.equals(I16view(arr)[1], -2)
asserts.equals(I8view(arr)[1], -2)

asserts.equals(I8view(arr)[2], -1)
asserts.equals(I32view(arr)[2], -1)

arr = Bytearray({0xFF, 0x7F})
local arri16 = I16view(arr)
local arru16 = U16view(arr)
asserts.equals(arri16[1], 32767)

arru16[1] = arru16[1] + 1
asserts.equals(arri16[1], -32768)
asserts.equals(arru16[1], 32768)

arr = Bytearray({0x9A, 0x99, 0x59, 0x40}) -- approx. equal 3.4 in hexadecimal representation of single-precision float number.
local arrflt = FLTview(arr)
local arru32 = U32view(arr)
asserts.equals(arrflt[1], 3.4000000953674316)
asserts.equals(arru32[1], 0x4059999A)

arrflt[1] = arrflt[1] + 0.1
asserts.equals(arrflt[1], 3.5)
asserts.equals(arru32[1], 0x40600000) -- hexadecimal representation of 3.5 single-precision float number.

arr = Bytearray({0x18, 0x2D, 0x44, 0x54, 0xFB, 0x21, 9, 64}) -- hexadecimal representation of 3.141592653589793 double-precision float number.
assert(DBLview(arr)[1] == 3.141592653589793)

-- =============================================

function barrtostr(barr)
    local outstr = ""
    for i = 1, #barr do
        local lastfmt = ""
        if i ~= #barr then lastfmt = "|" end
        outstr = outstr .. barr[i] .. lastfmt
    end
    return outstr
end

arr = Bytearray({0, 5, 34, 87, 21, 0, 210})
asserts.equals(barrtostr(arr), "0|5|34|87|21|0|210")
arr:move(2, 3, 4)
asserts.equals(barrtostr(arr), "0|5|5|34|87|21|210")

local arr2 = Bytearray({1, 2, 3, 4, 6, 7, 8})
asserts.equals(barrtostr(arr2), "1|2|3|4|6|7|8")
arr:copy(4, arr2, 2, 3)
asserts.equals(barrtostr(arr2), "1|34|87|21|6|7|8")

arr:fill(nil, nil, 192)
asserts.equals(barrtostr(arr), "192|192|192|192|192|192|192")
arr:fill(nil, 3, 233)
asserts.equals(barrtostr(arr), "233|233|233|233|233|233|233")
arr:fill(2, nil, 254)
asserts.equals(barrtostr(arr), "254|254|254|254|254|254|254")
arr:fill(2, 2, 66)
asserts.equals(barrtostr(arr), "254|66|66|254|254|254|254")
