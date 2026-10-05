#include "commons.hpp"
#include <sstream>
#include <stdexcept>

#include "util/stringutil.hpp"

parsing_error::parsing_error(
    const std::string& message,
    std::string_view filename,
    std::string_view source,
    uint pos,
    uint line,
    uint linestart
)
    : std::runtime_error(message),
      filename(filename),
      pos(pos),
      line(line),
      linestart(linestart) {
    size_t end = source.find("\n", linestart);
    if (end == std::string::npos) {
        end = source.length();
    }
    this->source = source.substr(linestart, end - linestart);
}

parsing_error::parsing_error(
    const std::string& message,
    std::string&& filename,
    std::string&& source,
    uint pos,
    uint line,
    uint linestart
)
    : std::runtime_error(message),
      filename(std::move(filename)),
      pos(pos),
      line(line),
      linestart(linestart) {
    size_t end = source.find("\n", linestart);
    if (end == std::string::npos) {
        end = source.length();
    }
    this->source = source.substr(linestart, end - linestart);
}

std::string parsing_error::errorLog() const {
    std::stringstream ss;
    uint linepos = pos - linestart;
    ss << "parsing error in file '" << filename;
    ss << "' at " << (line + 1) << ":" << linepos << ": " << this->what()
       << "\n";
    ss << source << "\n";
    for (uint i = 0; i < linepos; i++) {
        ss << " ";
    }
    ss << "^";
    return ss.str();
}

std::runtime_error parsing_error::toRuntimeError() const {
    return std::runtime_error("parsing error: " + errorLog());
}
