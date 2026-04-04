#include "Assets.hpp"

Assets::Assets(util::ObjectsKeeper* vault) : vault(vault) {
}

Assets::~Assets() {
    if (vault == nullptr) {
        return;
    }
    for (auto& [_, map] : assets) {
        for (auto& [__, asset] : map) {
            vault->keepAlive(std::move(asset));
        }
        map.clear();
    }
}

const std::vector<TextureAnimation>& Assets::getAnimations() {
    return animations;
}

void Assets::store(const TextureAnimation& animation) {
    animations.emplace_back(animation);
}
