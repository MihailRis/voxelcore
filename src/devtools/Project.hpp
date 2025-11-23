#pragma once

#include <string>
#include <vector>
#include <memory>

#include "interfaces/Process.hpp"
#include "interfaces/Serializable.hpp"

namespace scripting {
    class IClientProjectScript;
}

struct Project : Serializable {
    std::string name;
    std::string title;
    std::vector<std::string> basePacks;
    std::unique_ptr<scripting::IClientProjectScript> clientScript;
    std::unique_ptr<Process> setupCoroutine;

    ~Project();

    dv::value serialize() const override;
    void deserialize(const dv::value& src) override;

    void loadProjectClientScript();
    void loadProjectStartScript();
};
