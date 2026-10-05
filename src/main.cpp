#include "engine/Engine.hpp"
#include "util/platform.hpp"
#include "util/command_line.hpp"
#include "debug/Logger.hpp"

#include <csignal>
#include <iostream>
#include <stdexcept>

using namespace std::literals;
static debug::Logger logger("main");

static void sigterm_handler(int signum) {
    logger.info() << (signum == SIGTERM ? "SIGTERM" : "SIGINT") << " received";
    Engine::getInstance().quit();
}

int main(int argc, char** argv) {
#ifdef VC_BUILD_NAME
    if constexpr (VC_BUILD_NAME[0]) {
        logger.info() << "build: " << VC_BUILD_NAME;
    }
#endif

    CoreParameters coreParameters;
    try {
        if (!parse_cmdline(argc, argv, coreParameters)) {
            return EXIT_SUCCESS;
        }
        logger.debug() << "sub-process depth: "
                       << coreParameters.subProcessDepth;
    } catch (const std::runtime_error& err) {
        std::cerr << err.what() << std::endl;
        return EXIT_FAILURE;
    }
    std::signal(SIGTERM, sigterm_handler);
#ifdef NDEBUG
    std::signal(SIGINT, sigterm_handler);
#endif
    std::filesystem::path logFile = coreParameters.logFile;
    if (logFile.empty()) {
        logFile = coreParameters.userFolder.string() + "/latest"s +
                  (coreParameters.subProcessDepth > 0
                       ? ".sub" + std::to_string(coreParameters.subProcessDepth)
                       : "") + ".log"s;
    }
    debug::Logger::init(logFile.u8string());
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
