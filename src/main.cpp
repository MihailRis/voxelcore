#include "engine/Engine.hpp"
#include "util/platform.hpp"
#include "util/command_line.hpp"
#include "debug/Logger.hpp"

#include <csignal>
#include <iostream>
#include <stdexcept>

static debug::Logger logger("main");

static void sigterm_handler(int signum) {
    Engine::getInstance().quit();
}

int main(int argc, char** argv) {
    CoreParameters coreParameters;
    try {
        if (!parse_cmdline(argc, argv, coreParameters)) {
            return EXIT_SUCCESS;
        }
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        return EXIT_FAILURE;
    }
    std::signal(SIGTERM, sigterm_handler);
    
    debug::Logger::init(coreParameters.userFolder.string() + "/latest.log");
    platform::configure_encoding();

    auto& engine = Engine::getInstance();
    try {
        engine.initialize(std::move(coreParameters));
        engine.run();
    } catch (const initialize_error& err) {
        logger.error() << err.what();
        logger.error() << "could not to initialize engine";
    }
#if defined(NDEBUG) and defined(_WIN32)
    catch (const std::exception& err) {
        logger.error() << "uncaught exception: " << err.what();
        debug::Logger::flush();
        throw;
    }
#endif
    Engine::terminate();
    return EXIT_SUCCESS;
}
