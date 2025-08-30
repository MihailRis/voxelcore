#include "command_line.hpp"

#include <iostream>

#include "io/engine_paths.hpp"
#include "util/ArgsReader.hpp"
#include "engine/Engine.hpp"

namespace fs = std::filesystem;

static bool perform_keyword(
    util::ArgsReader& reader, const std::string& keyword, CoreParameters& params
) {
    if (keyword == "--res") {
        params.resFolder = reader.next();
    } else if (keyword == "--dir") {
        params.userFolder = reader.next();
    } else if (keyword == "--project") {
        params.projectFolder = reader.next();
    } else if (keyword == "--help" || keyword == "-h") {
        std::cout << "VoxelCore v" << ENGINE_VERSION_STRING << "\n\n";
        std::cout << "command-line arguments:\n";
        std::cout << " --help - display this help\n";
        std::cout << " --version - display engine version\n";
        std::cout << " --res <path> - set resources directory\n";
        std::cout << " --dir <path> - set userfiles directory\n";
        std::cout << " --project <path> - set project directory\n";
        std::cout << " --headless - run in headless mode\n";
        std::cout << " --test <path> - test script file\n";
        std::cout << " --script <path> - main script file\n";
        std::cout << std::endl;
        return false;
    } else if (keyword == "--version") {
        std::cout << ENGINE_VERSION_STRING << std::endl;
        return false;
    } else if (keyword == "--headless") {
        params.headless = true;
    } else if (keyword == "--test") {
        auto token = reader.next();
        params.testMode = true;
        params.scriptFile = token;
    } else if (keyword == "--script") {
        auto token = reader.next();
        params.testMode = false;
        params.scriptFile = token;
    } else {
        throw std::runtime_error("unknown argument " + keyword);
    }
    return true;
}

bool parse_cmdline(int argc, char** argv, CoreParameters& params) {
    util::ArgsReader reader(argc, argv);
    reader.skip();
    while (reader.hasNext()) {
        std::string token = reader.next();
        if (reader.isKeywordArg()) {
            if (!perform_keyword(reader, token, params)) {
                return false;
            }
        } else {
            std::cerr << "unexpected token" << std::endl;
            return false;
        }
    }
    return true;
}
