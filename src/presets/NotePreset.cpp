#define VC_ENABLE_REFLECTION

#include "NotePreset.hpp"
#include "data/dv_util.hpp"

#include <vector>

dv::value NotePreset::serialize() const {
    return dv::object({
        {"display", NoteDisplayModeMeta.getNameString(displayMode)},
        {"color", dv::to_value(color)},
        {"scale", scale},
        {"render_distance", renderDistance},
        {"xray_opacity", xrayOpacity},
        {"perspective", perspective},
    });
}

void NotePreset::deserialize(const dv::value& src) {
    if (src.has("display")) {
        NoteDisplayModeMeta.getItem(src["display"].asString(), displayMode);
    }
    if (src.has("color")) {
        dv::get_vec(src["color"], color);
    }
    src.at("scale").get(scale);
    src.at("render_distance").get(renderDistance);
    src.at("xray_opacity").get(xrayOpacity);
    src.at("perspective").get(perspective);
}
