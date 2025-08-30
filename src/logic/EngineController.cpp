#define VC_ENABLE_REFLECTION
#include "EngineController.hpp"

#include <algorithm>
#include <memory>

#include "engine/Engine.hpp"
#include "coders/commons.hpp"
#include "debug/Logger.hpp"
#include "coders/json.hpp"
#include "content/ContentReport.hpp"
#include "content/ContentControl.hpp"
#include "content/PacksManager.hpp"
#include "world/files/WorldConverter.hpp"
#include "world/files/WorldFiles.hpp"
#include "frontend/locale.hpp"
#include "frontend/menu.hpp"
#include "frontend/screens/LevelScreen.hpp"
#include "frontend/screens/MenuScreen.hpp"
#include "graphics/ui/GUI.hpp"
#include "graphics/ui/elements/Menu.hpp"
#include "graphics/ui/gui_util.hpp"
#include "objects/Players.hpp"
#include "interfaces/Task.hpp"
#include "util/stringutil.hpp"
#include "world/Level.hpp"
#include "world/World.hpp"
#include "LevelController.hpp"

static debug::Logger logger("engine-control");

EngineController::EngineController(Engine& engine) : engine(engine) {}

void EngineController::deleteWorld(const std::string& name) {
    io::path folder = engine.getPaths().getWorldFolderByName(name);

    auto deletion = [this, folder]() {
        logger.info() << "deleting " << folder.string();
        io::remove_all(folder);
        if (!engine.isHeadless()) {
            engine.getGUI().getMenu()->back();
        }
    };

    if (engine.isHeadless()) {
        deletion();
        return;
    }
    guiutil::confirm(
        engine,
        langs::get(L"delete-confirm", L"world") + L" (" +
            util::str2wstr_utf8(folder.string()) + L")",
        deletion
    );
}

std::shared_ptr<Task> create_converter(
    Engine& engine,
    const std::shared_ptr<WorldFiles>& worldFiles,
    const Content* content,
    const std::shared_ptr<ContentReport>& report,
    const runnable& postRunnable
) {
    ConvertMode mode;
    if (report->isUpgradeRequired()) {
        mode = ConvertMode::UPGRADE;
    } else if (report->hasContentReorder()) {
        mode = ConvertMode::REINDEX;
    } else {
        mode = ConvertMode::BLOCK_FIELDS;
    }
    return WorldConverter::startTask(
        worldFiles,
        content,
        report,
        [&engine, postRunnable]() {
            engine.postRunnable([=]() { postRunnable(); });
        },
        mode,
        true
    );
}

struct ConfirmRequest {
    bool memo;
    std::wstring message;
    std::wstring text;
};

static ConfirmRequest create_convert_request(
    const std::shared_ptr<ContentReport>& report
) {
    ConfirmRequest request {false, L"", L""};

    request.message = L"world.convert-block-layouts";
    if (report->hasContentReorder()) {
        request.message = L"world.convert-request";
    }
    if (report->isUpgradeRequired()) {
        request.message = L"world.upgrade-request";
    } else if (report->hasDataLoss()) {
        request.message = L"world.convert-with-loss";
        request.memo = true;
        for (const auto& line : report->getDataLoss()) {
            request.text += util::str2wstr_utf8(line) + L"\n";
        }
    }
    return request;
}

static void call(Engine& engine, runnable func) {
    if (engine.isHeadless()) {
        func();
    } else {
        menus::call(engine, std::move(func));
    }
}

static void start(Engine& engine, std::shared_ptr<Task> task, const std::wstring& message) {
    if (engine.isHeadless()) {
        task->waitForEnd();
    } else {
        menus::show_process_panel(engine, task, message);
    }
}

static void check_world(const EnginePaths& paths, const io::path& folder) {
    auto worldFile = folder / "world.json";
    if (!io::exists(worldFile)) {
        throw std::runtime_error(worldFile.string() + " does not exists");
    }
}

static const Content* load_world_content(Engine& engine, const io::path& folder) {
    auto& paths = engine.getPaths();
    auto& contentControl = engine.getContentControl();
    paths.setCurrentWorldFolder(folder);

    check_world(paths, folder);
    call(engine, [&contentControl]() {
        contentControl.loadContent(ContentPack::worldPacksList("world:"));
    });
    return contentControl.get();
}

static void load_world(
    Engine& engine,
    const std::shared_ptr<WorldFiles>& worldFiles,
    int64_t localPlayer
) {
    call(engine, [&engine, worldFiles, localPlayer] () {
        auto& contentControl = engine.getContentControl();
        auto content = contentControl.get();
        const auto& packs = contentControl.getContentPacks();
        auto& settings = engine.getSettings();

        auto level = World::load(worldFiles, settings, *content, packs);
        engine.onWorldOpen(std::move(level), localPlayer);
    });
}

static dv::value create_missing_content_report(
    const std::shared_ptr<ContentReport>& report
) {
    auto root = dv::object();
    auto& contentEntries = root.list("content");
    for (auto& entry : report->getMissingContent()) {
        std::string contentName = ContentTypeMeta.getNameString(entry.type);
        auto& contentEntry = contentEntries.object();
        contentEntry["type"] = contentName;
        contentEntry["name"] = entry.name;
    }
    return root;
}

void EngineController::onMissingContent(
    const std::shared_ptr<ContentReport>& report
) {
    if (engine.isHeadless()) {
        throw std::runtime_error(
            "missing content: " +
            json::stringify(create_missing_content_report(report), true, "  ")
        );
    } else {
        engine.setScreen(std::make_shared<MenuScreen>(engine));
        menus::show(
            engine,
            "reports/missing_content",
            {create_missing_content_report(report)}
        );
    }
}

static void confirm(
    Engine& engine, ConfirmRequest request, bool confirmed, runnable callback
) {
    if (confirmed || engine.isHeadless()) {
        callback();
        return;
    }
    if (request.memo) {
        guiutil::confirm_with_memo(
            engine,
            langs::get(request.message),
            request.text,
            callback,
            L"",
            langs::get(L"Cancel")
        );
    } else {
        guiutil::confirm(
            engine,
            langs::get(request.message),
            callback,
            nullptr,
            L"",
            langs::get(L"Cancel")
        );
    }
}

void EngineController::openWorld(const std::string& name, bool confirmConvert) {
    const auto& paths = engine.getPaths();
    auto& debugSettings = engine.getSettings().debug;
    auto folder = paths.getWorldsFolder() / name;

    auto content = load_world_content(engine, folder);
    auto worldFiles = std::make_shared<WorldFiles>(folder, debugSettings);
    auto report = World::checkIndices(worldFiles, content);
    
    if (report == nullptr) {
        load_world(engine, std::move(worldFiles), localPlayer);
        return;
    }
    if (report->hasMissingContent()) {
        onMissingContent(report);
        return;
    }

    auto request = create_convert_request(report);
    confirm(engine, std::move(request), confirmConvert, [=]() {
        auto task = create_converter(
            engine,
            worldFiles,
            content,
            report,
            [=]() { openWorld(name, false); }
        );
        start(engine, std::move(task), L"Converting world...");
    });
}

inline uint64_t str2seed(const std::string& seedstr) {
    if (util::is_integer(seedstr)) {
        try {
            return std::stoull(seedstr);
        } catch (const std::out_of_range& err) {
            std::hash<std::string> hash;
            return hash(seedstr);
        }
    } else {
        std::hash<std::string> hash;
        return hash(seedstr);
    }
}

void EngineController::createWorld(
    const std::string& name,
    const std::string& seedstr,
    const std::string& generatorID
) {
    uint64_t seed = str2seed(seedstr);

    EnginePaths& paths = engine.getPaths();
    auto folder = paths.getWorldsFolder() / name;

    call(engine, [this, &paths, folder]() {
        engine.getContentControl().loadContent();
        paths.setCurrentWorldFolder(folder);
    });

    auto& contentControl = engine.getContentControl();
    auto level = World::create(
        name,
        generatorID,
        folder,
        seed,
        engine.getSettings(),
        *contentControl.get(),
        contentControl.getContentPacks()
    );
    if (!engine.isHeadless()) {
        level->players->create(localPlayer);
    }
    engine.onWorldOpen(std::move(level), localPlayer);
}

void EngineController::setLocalPlayer(int64_t player) {
    localPlayer = player;
}

void EngineController::reopenWorld(World* world) {
    std::string name = world->wfile->getFolder().name();
    engine.onWorldClosed();
    openWorld(name, true);
}

static void reconfig_packs_outside(
    ContentControl& contentControl,
    const std::vector<std::string>& packsToAdd,
    const std::vector<std::string>& packsToRemove
) {
    auto& manager = contentControl.scan();

    auto names = PacksManager::getNames(
        contentControl.getContentPacks()
    );
    for (const auto& id : packsToAdd) {
        names.push_back(id);
    }
    for (const auto& id : packsToRemove) {
        manager.exclude(id);
        names.erase(std::find(names.begin(), names.end(), id));
    }
    names = manager.assemble(names);
    contentControl.setContentPacksRaw(manager.getAll(names));
}

static void reconfig_packs_inside(
    PacksManager& manager,
    std::vector<std::string>& names,
    const std::vector<std::string>& packsToAdd,
    const std::vector<std::string>& packsToRemove
) {
    for (const auto& id : packsToAdd) {
        names.push_back(id);
    }
    for (const auto& id : packsToRemove) {
        manager.exclude(id);
        const auto& found = std::find(names.begin(), names.end(), id);
        if (found != names.end()) {
            names.erase(found);
        } else {
            logger.warning()
                << "attempt to remove non-installed pack: " << id;
        }
    }
}

void EngineController::reconfigPacks(
    LevelController* controller,
    const std::vector<std::string>& packsToAdd,
    const std::vector<std::string>& packsToRemove
) {
    auto& contentControl = engine.getContentControl();
    auto content = contentControl.get();

    runnable removeFunc = [this, controller, packsToAdd, packsToRemove, &contentControl]() {
        if (controller == nullptr) {
            try {
                reconfig_packs_outside(contentControl, packsToAdd, packsToRemove);
            } catch (const contentpack_error& err) {
                throw std::runtime_error(
                    std::string(err.what()) + " [" + err.getPackId() + "]"
                );
            }
        } else {
            auto world = controller->getLevel()->getWorld();
            controller->saveWorld();

            auto names = PacksManager::getNames(world->getPacks());
            auto& manager = contentControl.scan();
            reconfig_packs_inside(manager, names, packsToAdd, packsToRemove);

            auto& wfile = *world->wfile;
            wfile.removeIndices(packsToRemove);
            wfile.writePacks(manager.getAll(names));
            reopenWorld(world);
        }
    };

    bool hasIndices = false;
    std::stringstream ss;
    if (content) {
        for (const auto& id : packsToRemove) {
            auto runtime = content->getPackRuntime(id);
            if (runtime && runtime->getStats().hasSavingContent()) {
                if (hasIndices) {
                    ss << ", ";
                }
                hasIndices = true;
                ss << id;
            }
        }
    }
    if (hasIndices && !engine.isHeadless()) {
        guiutil::confirm(
            engine,
            langs::get(L"remove-confirm", L"pack") + L" (" +
                util::str2wstr_utf8(ss.str()) + L")",
            [=]() { removeFunc(); }
        );
    } else {
        removeFunc();
    }
}
