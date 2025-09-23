#include <string>

#include "content/ContentPack.hpp"

class Version {
public:
    int major;
    int minor;
    int patch;

    Version(const std::string& version);

    bool operator==(const Version& other) const {
        return major == other.major && minor == other.minor &&
               patch == other.patch;
    }

    bool operator<(const Version& other) const {
        if (major != other.major) return major < other.major;
        if (minor != other.minor) return minor < other.minor;
        return patch < other.patch;
    }

    bool operator>(const Version& other) const {
        return other < *this;
    }

    bool operator>=(const Version& other) const {
        return !(*this < other);
    }

    bool operator<=(const Version& other) const {
        return !(*this > other);
    }

    bool processOperator(VersionOperator op, const Version& other) const {
        switch (op) {
            case VersionOperator::EQUAL:
                return *this == other;
            case VersionOperator::GREATHER:
                return *this > other;
            case VersionOperator::LESS:
                return *this < other;
            case VersionOperator::LESS_OR_EQUAL:
                return *this <= other;
            case VersionOperator::GREATHER_OR_EQUAL:
                return *this >= other;
            default:
                return false;
        }
    }

    static bool matchesPattern(const std::string& version);
};
