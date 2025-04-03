#pragma once

#include <string>

#include "data/dv.hpp"

namespace yaml {
    dv::value parse(std::string_view filename, std::string_view source);
    dv::value parse(std::string_view source);

    std::string stringify(const dv::value& value);
}
