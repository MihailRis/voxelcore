#pragma once

#include <string>
#include <vector>

#include "interfaces/Serializable.hpp"

struct Project : Serializable {
    std::string name;
    std::string title;
    std::vector<std::string> basePacks;

    dv::value serialize() const override;
    void deserialize(const dv::value& src) override;
};
