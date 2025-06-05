#pragma once

#include <memory>
#include <string>

namespace model {
    struct Model;
}

namespace vcm {
    std::unique_ptr<model::Model> parse(std::string_view file, std::string_view src);
}
