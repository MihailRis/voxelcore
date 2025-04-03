#include "Weather.hpp"

#include "data/dv.hpp"

dv::value Weather::serialize() const {
    return dv::object({
        {"a", a.serialize()},
        {"b", b.serialize()},
        {"name-a", nameA},
        {"name-b", nameB},
        {"t", t},
        {"speed", speed},
    });
}

void Weather::deserialize(const dv::value& src) {
    if (src.has("a")) {
        a.deserialize(src["a"]);
    }
    if (src.has("b")) {
        b.deserialize(src["b"]);
    }
    src.at("name-a").get(nameA);
    src.at("name-b").get(nameB);
    src.at("t").get(t);
    src.at("speed").get(speed);
}
