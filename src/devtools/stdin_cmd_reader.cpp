#include "stdin_cmd_reader.hpp"

#include "engine/Engine.hpp"
#include "logic/CommandsInterpreter.hpp"
#include "coders/json.hpp"
#include "debug/Logger.hpp"
#include "util/platform.hpp"

#include <thread>
#include <iostream>

static debug::Logger logger("stdin-cmd");

static std::thread reader_thread;

void cmd::start_stdin_cmd_reader(Engine& engine) {
    reader_thread = std::thread([&engine]() {
        auto& interpreter = engine.getCmd();
        logger.info() << "reader thread started";
        
        std::string line;
        while (true) {
            if (!platform::stdin_has_data()) {
                platform::sleep(10);
                continue;
            }
            std::getline(std::cin, line);
            if (line.empty()) {
                continue;
            }
            engine.postRunnable([line, &interpreter] () {
                try {
                    auto result = interpreter.execute(line);
                    if (result.isString()) {
                        logger.info() << result.asString();
                    } else {
                        logger.info() << json::stringify(result, true);
                    }
                } catch (const std::exception& err) {
                    logger.error() << err.what();
                }
            });
        }
    });
    reader_thread.detach();
}
