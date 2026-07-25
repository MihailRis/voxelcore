#pragma once

#include "interfaces/Serializable.hpp"

#include <set>
#include <string>
#include <vector>
#include <memory>

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
    Permissions permissions;

    ~Project();

    dv::value serialize() const override;
    void deserialize(const dv::value& src) override;
};
