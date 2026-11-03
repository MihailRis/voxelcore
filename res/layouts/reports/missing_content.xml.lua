content_report = nil

function delete_content()
    local units = {}
    for i, entry in ipairs(content_report.content) do
        table.insert(units, entry.name)
    end
    local request = {
        units = units
    }
    app.delete_world_content(content_report.world, request, true)
end

function on_open(report)
    content_report = report
    for i, entry in ipairs(report.content) do
        document.content_panel:add(gui.template("content_entry", entry))
    end
end
