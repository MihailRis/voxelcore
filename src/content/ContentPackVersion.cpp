#include "ContentPackVersion.hpp"

#include <algorithm>
#include <iostream>
#include <sstream>

#include "coders/commons.hpp"

Version::Version(const std::string& version) {
    major = 0;
    minor = 0;
    patch = 0;

    std::vector<int> parts;

    std::stringstream ss(version);
    std::string part;
    while (std::getline(ss, part, '.')) {
        if (!part.empty()) {
            parts.push_back(std::stoi(part));
        }
    }

    if (parts.size() > 0) major = parts[0];
    if (parts.size() > 1) minor = parts[1];
    if (parts.size() > 2) patch = parts[2];
}

bool isNumber(const std::string& s) {
    return !s.empty() && std::all_of(s.begin(), s.end(), ::is_digit);
}

bool Version::matchesPattern(const std::string& version) {
    for (char c : version) {
        if (!isdigit(c) && c != '.') {
            return false;
        }
    }

    std::stringstream ss(version);

    std::vector<std::string> parts;
    std::string part;
    while (std::getline(ss, part, '.')) {
        if (part.empty()) return false;
        if (!isNumber(part)) return false;

        parts.push_back(part);
    }

    return parts.size() == 2 || parts.size() == 3;
}
