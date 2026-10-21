#pragma once

#include <memory>
#include <string>

#include "typedefs.hpp"
#include "util/Buffer.hpp"

class Texture;
class ImageData;
namespace io {
    class path;
}

namespace png {
    std::unique_ptr<ImageData> load_image(const ubyte* bytes, size_t size);
    void write_image(const io::path& file, const ImageData* image);
    util::Buffer<ubyte> encode_image(const ImageData& image);
    std::unique_ptr<Texture> load_texture(const ubyte* bytes, size_t size);
    std::unique_ptr<Texture> load_texture(const std::string& filename);
}
