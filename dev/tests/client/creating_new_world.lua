app.tick()
app.set_setting("ui.language", "en_US")

local main_page = Document.new("core:pages/main")
test.click(main_page.worlds_btn)

local worlds_page = Document.new("core:pages/worlds")
test.click(worlds_page.new_world_btn)

local page = Document.new("core:pages/new_world")
local world_name = "test-"..random.uuid()
test.fill(page.name_box, world_name)
test.fill(page.seed_box, "randomseed")
test.click(page.generator_btn)

local generators_page = Document.new("core:pages/generators")
test.click(test.find_by_text(generators_page.root, "Flat_grass"))

local new_world_page = Document.new("core:pages/new_world")
test.click(new_world_page.create_world_btn)

app.sleep_until(function() return gui.root["hud.hotbar"].exists end)

test.press("escape")
local pause_page = Document.new("core:pages/pause")
test.click(pause_page.quit_btn)

test.click(main_page.worlds_btn)

local world_button = test.find_by_text(worlds_page.root, world_name).parent
local delete_button = test.find_by_attr(world_button, "src", "gui/delete_icon")
test.click(delete_button)

assert(test.find_by_text(
    gui.root.root, string.format("Do you want to delete world forever? (user:worlds/%s)", world_name)
), "World deletion confirm dialog not shown")

test.press("enter")

assert(not world_button.exists, "World still shown in worlds list after deletion")
