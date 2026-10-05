#pragma once

#include <stdexcept>
#include <string>

#include "typedefs.hpp"

inline bool is_digit(int c) {
    return (c >= '0' && c <= '9');
}

inline bool is_whitespace(int c) {
    return c == ' ' || c == '\n' || c == '\r' || c == '\t' || c == '\f';
}

inline bool is_identifier_start(int c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '_';
}

inline bool is_identifier_part(int c) {
    return is_identifier_start(c) || is_digit(c) || c == '-';
}

inline bool is_minimal_identifier_part(int c) {
    return is_identifier_start(c) || is_digit(c);
}

inline bool is_json_identifier_start(int c) {
    return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || c == '_' ||
           c == '.';
}

inline bool is_json_identifier_part(int c) {
    return is_json_identifier_start(c) || is_digit(c) || c == '-';
}

inline int hexchar2int(int c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return 10 + c - 'a';
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + c - 'A';
    }
    return -1;
}

class parsing_error : public std::runtime_error {
public:
    std::string filename;
    std::string source;
    uint pos;
    uint line;
    uint linestart;
    
    parsing_error(
        const std::string& message,
        std::string_view filename,
        std::string_view source,
        uint pos,
        uint line,
        uint linestart
    );

    parsing_error(
        const std::string& message,
        std::string&& filename,
        std::string&& source,
        uint pos,
        uint line,
        uint linestart
    );
    std::string errorLog() const;

    std::runtime_error toRuntimeError() const;
};
