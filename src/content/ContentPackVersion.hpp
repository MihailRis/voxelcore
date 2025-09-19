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

    bool process_operator(const std::string& op, const Version& other) const {
        auto dep_op = Version::string_to_operator(op);

        switch (dep_op) {
            case DependencyVersionOperator::EQUAL:
                return *this == other;
            case DependencyVersionOperator::GREATHER:
                return *this > other;
            case DependencyVersionOperator::LESS:
                return *this < other;
            case DependencyVersionOperator::LESS_OR_EQUAL:
                return *this <= other;
            case DependencyVersionOperator::GREATHER_OR_EQUAL:
                return *this >= other;
            default:
                return false;
        }
    }

    static DependencyVersionOperator string_to_operator(const std::string& op);
    static bool matches_pattern(const std::string& version);
};
