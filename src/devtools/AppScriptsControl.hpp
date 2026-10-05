#pragma once

#include "interfaces/Process.hpp"

#include <memory>
#include <string>

namespace scripting {
    class IClientProjectScript;
}

struct CoreParameters;
struct Project;

class AppScriptsControl {
public:
    AppScriptsControl(const CoreParameters& params);

    void tick();
    void loadProjectClientScript();
    void terminate(std::string_view reason);

    void onScreenChange(const std::string& name, bool show);

    bool isFinished() const;
private:
    std::unique_ptr<scripting::IClientProjectScript> clientScript;
    std::unique_ptr<Process> scriptCoroutine;
};
