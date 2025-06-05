#pragma once

#include <sstream>

namespace debug {
    enum class LogLevel { print, debug, info, warning, error };

    class Logger;

    class LogMessage {
        Logger* logger;
        LogLevel level;
        std::stringstream ss;
    public:
        LogMessage(Logger* logger, LogLevel level)
            : logger(logger), level(level) {
        }
        ~LogMessage();

        template <class T>
        LogMessage& operator<<(const T& x) {
            ss << x;
            return *this;
        }
    };

    class Logger {
        std::string name;
    public:
        static void init(const std::string& filename);
        static void flush();

        Logger(const std::string& name) : name(name) {
        }

        void log(LogLevel level, std::string message);
        
        LogMessage debug() {
            return LogMessage(this, LogLevel::debug);
        }

        LogMessage info() {
            return LogMessage(this, LogLevel::info);
        }

        LogMessage error() {
            return LogMessage(this, LogLevel::error);
        }

        LogMessage warning() {
            return LogMessage(this, LogLevel::warning);
        }
        
        /// @brief Print-debugging tool (printed without header)
        LogMessage print() {
            return LogMessage(this, LogLevel::print);
        }
    };
}
