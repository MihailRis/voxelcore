#include "command_line.hpp"

#include <iostream>
#include <functional>
#include <vector>
#include <string>

#include "io/engine_paths.hpp"
#include "util/ArgsReader.hpp"
#include "engine/Engine.hpp"

namespace fs = std::filesystem;

class ArgC {
    public:
        std::string keyword;
        std::function<bool()> execute;
        std::string help;
        ArgC(const std::string& keyword, std::function<bool()> execute, const std::string& help) {
            this->keyword = keyword;
            this->execute = execute;
            this->help = help;
        }
};


static bool perform_keyword(
    util::ArgsReader& reader, const std::string& keyword, CoreParameters& params
) {
    static const std::vector<ArgC> argumentsCommandline = {
        ArgC("--res", [&params, &reader]() -> bool {
            params.resFolder = reader.next();
            return true;
        }, "<path> - set resources directory."),
        ArgC("--dir", [&params, &reader]() -> bool {
            params.userFolder = reader.next();
            return true;
        }, "<path> - set userfiles directory."),
        ArgC("--project", [&params, &reader]() -> bool {
            params.projectFolder = reader.next();
            return true;
        }, "<path> - set project directory."),
        ArgC("--test", [&params, &reader]() -> bool {
            params.testMode = true;
            params.scriptFile = reader.next();
            return true;
        }, "<path> - test script file."),
        ArgC("--script", [&params, &reader]() -> bool {
            params.testMode = false;
            params.scriptFile = reader.next();
            return true;
        }, "<path> - main script file."),
        ArgC("--headless", [&params]() -> bool {
            params.headless = true;
            return true;
        }, "- run in headless mode."),
        ArgC("--version", []() -> bool {
            std::cout << ENGINE_VERSION_STRING << std::endl;
            return false;
        }, "- display the engine version."),
        ArgC("--help", []() -> bool {
            std::cout << "VoxelCore v" << ENGINE_VERSION_STRING << "\n\n";
            std::cout << "Command-line arguments:\n";
            for (auto& a : argumentsCommandline) {
                std::cout << a.keyword << " " << a.help << std::endl;
            }
            std::cout << std::endl;
            return false;
        }, "- display this help.")
    };
    for (auto& a : argumentsCommandline) {
        if (a.keyword == keyword) {
            return a.execute();
        }
    }
    throw std::runtime_error("unknown argument " + keyword);
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
