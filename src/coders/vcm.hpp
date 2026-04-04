#pragma once

#include <memory>
#include <string>
#include <optional>
#include <unordered_map>

#include "graphics/commons/Model.hpp"
#include "objects/rigging.hpp"

namespace rigging {
    class SkeletonConfig;
}

namespace vcm {
    struct VcmModel {
        std::unordered_map<std::string, model::Model> parts;
        std::optional<rigging::SkeletonConfig> skeleton;

        model::Model& squash();
        model::Model squashed() const;
    };

    VcmModel parse(
        std::string_view file, std::string_view src, bool usexml
    );
}
