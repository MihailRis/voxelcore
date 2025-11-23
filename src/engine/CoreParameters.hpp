#pragma once

#include <string>
#include <filesystem>

struct CoreParameters {
    bool headless = false;
    bool testMode = false;
    std::filesystem::path resFolder = "res";
    std::filesystem::path userFolder = ".";
    std::filesystem::path scriptFile;
    std::filesystem::path projectFolder;
    std::string debugServerString;
    int tps = 20;
};
