content_report = nil

function delete_content()
    local blocks = {}
    local items = {}
    for i, entry in ipairs(content_report.content) do
        local type = entry.type
        if type == "block" then
            table.insert(blocks, entry.name)
        elseif type == "item" then
            table.insert(items, entry.name)
        end
    end
    local request = {
        blocks = blocks,
        items = items
    }
    app.delete_world_content(content_report.world, request, true)
end

function on_open(report)
    content_report = report
    for i, entry in ipairs(report.content) do
        document.content_panel:add(gui.template("content_entry", entry))
    end
end
