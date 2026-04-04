#pragma once

#include <set>
#include <string>
#include <vector>
#include <memory>

#include "interfaces/Process.hpp"
#include "interfaces/Serializable.hpp"

namespace scripting {
    class IClientProjectScript;
}

struct Permissions {
    static inline std::string DEBUGGING = "debugging";
    static inline std::string NETWORK = "network";
    static inline std::string RECORD_AUDIO = "record-audio";
    static inline std::string WRITE_TO_USER = "write-to-user";

    std::set<std::string> permissions;

    bool has(const std::string& name) const;
};

struct Project : Serializable {
    std::string name;
    std::string title;
    std::vector<std::string> basePacks;
    std::unique_ptr<scripting::IClientProjectScript> clientScript;
    std::unique_ptr<Process> setupCoroutine;
    Permissions permissions;

    ~Project();

    dv::value serialize() const override;
    void deserialize(const dv::value& src) override;

    void loadProjectClientScript();
    void loadProjectStartScript();
};
