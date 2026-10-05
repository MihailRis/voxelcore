local src = "<test><sub>test</sub></test>"
local parsed = xml.parse(src)
asserts.equals(src, xml.tostring(parsed, false))

local src = "{\"test\": \"test\", \"sub\": {\"test\": \"test\"}}"
local parsed = json.parse(src)
asserts.equals("test", parsed.test)
asserts.equals("test", parsed.sub.test)

-- toml
local src = [[
[section]
key = "value"
# comment
key2 = "other value"
number = 42
pi = 3.14159
enabled = true
items = ["a", "b", "c"]
table = { x = 1, y = 2 }
]]
local parsed = toml.parse(src)

asserts.equals("value", parsed.section.key)
asserts.equals("other value", parsed.section.key2)
asserts.equals(42, parsed.section.number)
asserts.equals(3.14159, parsed.section.pi)
asserts.equals(true, parsed.section.enabled)
asserts.equals("a", parsed.section.items[1])
asserts.equals("b", parsed.section.items[2])
asserts.equals("c", parsed.section.items[3])
asserts.equals(1, parsed.section.table.x)
asserts.equals(2, parsed.section.table.y)
