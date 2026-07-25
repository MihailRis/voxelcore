#include "AppScriptsControl.hpp"

#include "debug/Logger.hpp"
#include "engine/CoreParameters.hpp"
#include "io/io.hpp"
#include "Project.hpp"
#include "logic/scripting/scripting.hpp"

static debug::Logger logger("app-scripts");

AppScriptsControl::AppScriptsControl(
    const CoreParameters& params, const Project& project
)
    : project(project) {

    io::path scriptFile =
        params.scriptFile.empty()
            ? "project:start.lua"
            : std::string("script:") + params.scriptFile.filename().u8string();
    if (io::exists(scriptFile)) {
        logger.info() << "starting script: " << params.scriptFile.u8string();
        scriptCoroutine = scripting::start_app_script(scriptFile);
    } else {
        logger.warning() << "script does not exists: " << params.scriptFile.u8string();
    }

    if (!params.headless) {
        loadProjectClientScript();
    }
}

void AppScriptsControl::loadProjectClientScript() {
    io::path scriptFile = "project:project_client.lua";
    if (io::exists(scriptFile)) {
        logger.info() << "starting project client script: " << scriptFile.string();
        clientScript = scripting::load_client_project_script(scriptFile);
    } else {
        logger.warning() << "project client script does not exists: " << scriptFile.string();
    }
}

void AppScriptsControl::onScreenChange(const std::string& name, bool show) {
    if (clientScript) {
        clientScript->onScreenChange(name, show);
    }
}

void AppScriptsControl::tick() {
    if (scriptCoroutine && scriptCoroutine->isActive()) {
        scriptCoroutine->update();
    }
}

void AppScriptsControl::terminate(std::string_view reason) {
    if (scriptCoroutine->isActive()) {
        scriptCoroutine->terminate();
        logger.info() << "script has been terminated due to " << reason;
    }
}

bool AppScriptsControl::isFinished() const {
    return scriptCoroutine == nullptr || !scriptCoroutine->isActive();
}
