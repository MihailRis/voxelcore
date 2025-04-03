#include <gtest/gtest.h>

#include "coders/yaml.hpp"
#include "coders/json.hpp"
#include "coders/commons.hpp"

#include "io/io.hpp"
#include "io/devices/StdfsDevice.hpp"

namespace fs = std::filesystem;

TEST(YAML, EncodeDecode) {
    io::set_device("root", std::make_shared<io::StdfsDevice>(fs::u8path("../../")));
    auto filename = "root:.github/workflows/windows-clang.yml";
    try {
        auto value = yaml::parse(io::read_string(filename));
        std::cout << yaml::stringify(value) << std::endl;
    } catch (const parsing_error& error) {
        std::cerr << error.errorLog() << std::endl;
        throw error;
    }
}
