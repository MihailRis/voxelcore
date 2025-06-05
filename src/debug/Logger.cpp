#include "Logger.hpp"

#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <utility>

using namespace debug;

static std::ofstream file;
static std::mutex mutex;
static std::string utcOffset = "";
constexpr unsigned int moduleLen = 20;

LogMessage::~LogMessage() {
    logger->log(level, ss.str());
}

static void write(
    LogLevel level, const std::string& name, const std::string& message
) {
    if (level == LogLevel::print) {
        std::cout << "[" << name << "]    " << message << std::endl;
        return;
    }

    using namespace std::chrono;

    std::stringstream ss;
    switch (level) {
        case LogLevel::print:
        case LogLevel::debug:
#ifdef NDEBUG
            return;
#endif
            ss << "[D]";
            break;
        case LogLevel::info:
            ss << "[I]";
            break;
        case LogLevel::warning:
            ss << "[W]";
            break;
        case LogLevel::error:
            ss << "[E]";
            break;
    }
    time_t tm = std::time(nullptr);
    auto ms =
        duration_cast<milliseconds>(system_clock::now().time_since_epoch()) %
        1000;
    ss << " " << std::put_time(std::localtime(&tm), "%Y/%m/%d %T");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    ss << utcOffset << " [" << std::setfill(' ') << std::setw(moduleLen) << name
       << "] ";
    ss << message;
    {
        std::lock_guard<std::mutex> lock(mutex);
        auto string = ss.str();
        if (file.good()) {
            file << string << '\n';
            file.flush();
        }
        std::cout << string << std::endl;
    }
}

void Logger::init(const std::string& filename) {
    file.open(filename);

    time_t tm = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&tm), "%z");
    utcOffset = ss.str();
}

void Logger::flush() {
    std::lock_guard<std::mutex> lock(mutex);
    file.flush();
}

void Logger::log(LogLevel level, std::string message) {
    write(level, name, std::move(message));
}
