#define VC_ENABLE_REFLECTION
#include "Environment.hpp"

#include "data/dv.hpp"
#include "data/dv_util.hpp"

dv::value Environment::serialize() const {
    auto skySpritesList = dv::list();
    for (const auto& sprite : sky.sprites) {
        skySpritesList.add(dv::object({
            {"texture", sprite.texture},
            {"phase", sprite.phase},
            {"distance", sprite.distance},
            {"emissive", sprite.emissive},
            {"altitude", sprite.altitude},
        }));
    }
    return dv::object({
        {"sky", dv::object({
            {"mode", SkyModeMeta.getNameString(sky.mode)},
            {"stars", sky.stars},
            {"clouds", sky.clouds},
            {"sprites", std::move(skySpritesList)},
        })},
        {"generator", generator}
    });
}

void Environment::deserialize(const dv::value& src) {
    if (auto skyItem = src.at("sky")) {
        skyItem->at("mode").get(sky.mode, SkyModeMeta);
        skyItem->at("stars").get(sky.stars);
        skyItem->at("clouds").get(sky.clouds);

        if (auto skySpritesList = skyItem->at("sprites")) {
            for (int i = 0; i < skySpritesList->size(); i++) {
                SkySprite sprite;
                const auto& item = (*skySpritesList.ptr)[i];
                item.at("texture").get(sprite.texture);
                item.at("phase").get(sprite.phase);
                item.at("distance").get(sprite.distance);
                item.at("emissive").get(sprite.emissive);
                item.at("altitude").get(sprite.altitude);
                sky.sprites.push_back(std::move(sprite));
            }
        }
    }
    src.at("generator").get(generator);
}
