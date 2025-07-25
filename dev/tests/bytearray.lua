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
assert(#arr == 13)
for i=1,10 do
    assert(arr[i] == 10 - i)
end
print(#arr, arr:get_capacity())
arr:trim()
assert(#arr == arr:get_capacity())

arr = Bytearray({0, 2, 7, 1, 16, 75, 25})
assert(arr[6] == 75)
