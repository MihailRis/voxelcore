#include "NamedSkeletons.hpp"

#include "objects/rigging.hpp"

using namespace rigging;

NamedSkeletons::NamedSkeletons() = default;

std::shared_ptr<rigging::Skeleton> NamedSkeletons::createSkeleton(
    const std::string& name, const SkeletonConfig* config
) {
    auto skeleton = std::make_shared<Skeleton>(config);
    skeletons[name] = skeleton;
    return skeleton;
}

rigging::Skeleton* NamedSkeletons::getSkeleton(const std::string& name) {
    const auto& found = skeletons.find(name);
    if (found == skeletons.end()) {
        return nullptr;
    }
    return found->second.get();
}
