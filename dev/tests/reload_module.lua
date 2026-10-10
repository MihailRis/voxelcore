-- reload_module must run the module in its pack environment, as require does
local util = require "core:tests_util"
util.create_demo_world()

-- a pack with a module that uses pack environment (PACK_ID)
local PACK = "world:content/reload_test"
file.mkdirs(PACK .. "/modules")
file.write(PACK .. "/package.json", json.tostring({id="reload_test", title="reload_test"}))

local function write_module(version)
    file.write(PACK .. "/modules/mod.lua", string.format([[
local this = {}
function this.info() return PACK_ID, %d end
return this
]], version))
end

write_module(1)
app.reconfig_packs({"reload_test"}, {})
app.tick()

local mod = require "reload_test:mod"
local packid, version = mod.info()
assert(packid == "reload_test" and version == 1)

write_module(2)
reload_module("reload_test:mod")
packid, version = mod.info()
assert(version == 2, "module is not reloaded")
assert(packid == "reload_test", "reloaded module has no pack environment")

app.close_world(true)
app.delete_world("demo")
