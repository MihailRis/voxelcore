#pragma once

#include <string>
#include <filesystem>
#include <unordered_map>

struct CoreParameters {
    bool headless = false;
    bool testMode = false;
    bool stdinCommands = false;
    std::filesystem::path resFolder = "res";
    std::filesystem::path userFolder = ".";
    std::filesystem::path scriptFile;
    std::filesystem::path projectFolder;
    std::string debugServerString;
    int tps = 20;
    std::unordered_map<std::string, std::string> projectArgs;
};
