#include "menu.hpp"

#include <glm/glm.hpp>

#include "UiDocument.hpp"
#include "coders/commons.hpp"
#include "data/dv.hpp"
#include "delegates.hpp"
#include "engine/Engine.hpp"
#include "graphics/ui/GUI.hpp"
#include "graphics/ui/elements/Menu.hpp"
#include "graphics/ui/gui_util.hpp"
#include "interfaces/Task.hpp"
#include "engine/EnginePaths.hpp"
#include "locale.hpp"
#include "logic/scripting/scripting.hpp"
#include "screens/MenuScreen.hpp"
#include "settings.hpp"
#include "util/stringutil.hpp"
#include "assets/assetload_funcs.hpp"
#include "content/ContentPack.hpp"

using namespace gui;

void menus::create_version_label(gui::GUI& gui) {
    auto text = ENGINE_VERSION_STRING + " debug build";
    gui.add(guiutil::create(
        gui,
        "<label z-index='1000' color='#FFFFFF30' gravity='top-right' "
        "margin='4'>" +
        text + "</label>"
    ));
}

void menus::call(Engine& engine, runnable func) {
    if (engine.isHeadless()) {
        throw std::runtime_error("menus::call(...) in headless mode");
    }
    try {
        func();
    } catch (const contentpack_error& error) {
        engine.setScreen(std::make_shared<MenuScreen>(engine));
        // could not to find or read pack
        guiutil::alert(
            engine,
            langs::get(L"error.pack-not-found") + L": " +
                util::str2wstr_utf8(error.getPackId())
        );
        throw std::runtime_error(error);
    } catch (const assetload::error& error) {
        engine.setScreen(std::make_shared<MenuScreen>(engine));
        guiutil::alert(
            engine,
            langs::get(L"Assets Load Error", L"menu") + L":\n" +
            util::str2wstr_utf8(error.what())
        );
        throw std::runtime_error(error);
    } catch (const parsing_error& error) {
        engine.setScreen(std::make_shared<MenuScreen>(engine));
        guiutil::alert(engine, util::str2wstr_utf8(error.errorLog()));
        throw std::runtime_error(error);
    } catch (const std::runtime_error& error) {
        engine.setScreen(std::make_shared<MenuScreen>(engine));
        guiutil::alert(
            engine,
            langs::get(L"Content Error", L"menu") + L":\n" +
            util::str2wstr_utf8(error.what())
        );
        throw std::runtime_error(error);
    }
}

UiDocument* menus::show(
    Engine& engine, const std::string& name, std::vector<dv::value> args
) {
    auto menu = engine.getGUI().getMenu();
    auto file = engine.getResPaths().find("layouts/" + name + ".xml");
    auto fullname = "core:layouts/" + name;

    auto documentPtr = UiDocument::read(
        engine.getGUI(),
        scripting::get_root_environment(),
        fullname,
        file,
        "core:layouts/" + name
    );
    auto document = documentPtr.get();
    engine.getAssets()->store(std::move(documentPtr), fullname);
    scripting::on_ui_open(document, std::move(args));
    menu->addPage(name, document->getRoot());
    menu->setPage(name);
    return document;
}

void menus::show_process_panel(
    Engine& engine, const std::shared_ptr<Task>& task, const std::wstring& text
) {
    uint initialWork = task->getWorkTotal();

    auto menu = engine.getGUI().getMenu();
    menu->reset();
    auto doc =
        menus::show(engine, "process", {util::wstr2str_utf8(langs::get(text))});
    std::dynamic_pointer_cast<Container>(doc->getRoot())
        ->listenInterval(0.01f, [=]() {
            task->update();

            uint tasksDone = task->getWorkDone();
            scripting::on_ui_progress(doc, tasksDone, initialWork);
        });
}
