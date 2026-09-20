#include <gtest/gtest.h>

#include "network/Network.hpp"
#include "coders/json.hpp"

TEST(curltest, curltest) {
    NetworkSettings settings {};
    auto network = network::Network::create(settings);

    network::HttpRequest request {};
    request.url = "https://raw.githubusercontent.com/MihailRis/VoxelEngine-Cpp/refs/"
                  "heads/curl/res/content/base/blocks/lamp.json";
    request.onResponse = [](network::HttpResponse response) {
        if (response.body.empty()) {
            return;
        }
        auto view =
            std::string_view(response.body.data(), response.body.size());
        auto value = json::parse(view);
        std::cout << value << std::endl;
    };
    network->request(std::move(request));
    
    std::cout << "upload: " << network->getTotalUpload() << " B" << std::endl;
    std::cout << "download: " << network->getTotalDownload() << " B" << std::endl;
}
