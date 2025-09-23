#include <gtest/gtest.h>

#include "network/Network.hpp"
#include "coders/json.hpp"

TEST(curltest, curltest) {
    NetworkSettings settings {};
    auto network = network::Network::create(settings);
    network->get(
        "https://raw.githubusercontent.com/MihailRis/VoxelEngine-Cpp/refs/"
        "heads/curl/res/content/base/blocks/lamp.json",
        [](std::vector<char> data) {
            if (data.empty()) {
                return;
            }
            auto view = std::string_view(data.data(), data.size());
            auto value = json::parse(view);
            std::cout << value << std::endl;
        }, [](auto, auto){}
    );
    std::cout << "upload: " << network->getTotalUpload() << " B" << std::endl;
    std::cout << "download: " << network->getTotalDownload() << " B" << std::endl;
}
