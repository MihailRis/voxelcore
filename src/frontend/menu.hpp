#pragma once

#include "data/dv.hpp"
#include "delegates.hpp"

#include <string>
#include <vector>
#include <memory>

class Task;
class Engine;

class UiDocument;

namespace gui {
    class GUI;
}

namespace menus {
    /// @brief Create development version label at the top-right screen corner
    void create_version_label(gui::GUI& gui);

    UiDocument* show(
        Engine& engine, 
        const std::string& name,
        std::vector<dv::value> args
    );

    void show_process_panel(
        Engine& engine,
        const std::shared_ptr<Task>& task,
        const std::wstring& text = L""
    );

    void call(Engine& engine, runnable func);
}
