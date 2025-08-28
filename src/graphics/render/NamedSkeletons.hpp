#pragma once

#include <memory>
#include <string>
#include <unordered_map>

namespace rigging {
    struct Skeleton;
    class SkeletonConfig;
}

class NamedSkeletons {
public:
    NamedSkeletons();

    std::shared_ptr<rigging::Skeleton> createSkeleton(
        const std::string& name, const rigging::SkeletonConfig* config
    );

    rigging::Skeleton* getSkeleton(const std::string& name);
private:
    std::unordered_map<std::string, std::shared_ptr<rigging::Skeleton>> skeletons;
};
