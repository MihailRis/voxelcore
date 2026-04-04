#define VC_ENABLE_REFLECTION
#include "imageio.hpp"

#include <functional>
#include <unordered_map>

#include "graphics/core/ImageData.hpp"
#include "io/io.hpp"
#include "png.hpp"

using namespace imageio;

using image_reader =
    std::function<std::unique_ptr<ImageData>(const ubyte*, size_t)>;
using image_writer = std::function<void(const std::string&, const ImageData*)>;

static std::unordered_map<ImageFileFormat, image_reader> readers {
    {ImageFileFormat::PNG, png::load_image},
};

static std::unordered_map<ImageFileFormat, image_writer> writers {
    {ImageFileFormat::PNG, png::write_image},
};

bool imageio::is_read_supported(const std::string& extension) {
    return extension == ".png";
}

bool imageio::is_write_supported(const std::string& extension) {
    return extension == ".png";
}

std::unique_ptr<ImageData> imageio::read(const io::path& file) {
    ImageFileFormat format;
    if (!ImageFileFormatMeta.getItem(file.extension().substr(1), format)) {
        throw std::runtime_error("unsupported image format");
    }
    auto found = readers.find(format);
    if (found == readers.end()) {
        throw std::runtime_error(
            "file format is not supported (read): " + file.string()
        );
    }
    auto bytes = io::read_bytes_buffer(file);
    try {
        return std::unique_ptr<ImageData>(found->second(bytes.data(), bytes.size()));
    } catch (const std::runtime_error& err) {
        throw std::runtime_error(
            "could not to load image " + file.string() + ": " + err.what()
        );
    }
}

std::unique_ptr<ImageData> imageio::decode(
    ImageFileFormat format, util::span<ubyte> src
) {
    auto found = readers.find(format);
    try {
        return std::unique_ptr<ImageData>(found->second(src.data(), src.size()));
    } catch (const std::runtime_error& err) {
        throw std::runtime_error(
            "could not to decode image: " + std::string(err.what())
        );
    }
}

void imageio::write(const io::path& file, const ImageData* image) {
    ImageFileFormat format;
    if (!ImageFileFormatMeta.getItem(file.extension().substr(1), format)) {
        throw std::runtime_error("unsupported image format");
    }
    auto found = writers.find(format);
    if (found == writers.end()) {
        throw std::runtime_error(
            "file format is not supported (write): " + file.string()
        );
    }
    return found->second(io::resolve(file).u8string(), image);
}

util::Buffer<unsigned char> imageio::encode(
    ImageFileFormat format, const ImageData& image
) {
    switch (format) {
        case ImageFileFormat::PNG:
            return png::encode_image(image);
        default:
            throw std::runtime_error("file format is not supported for encoding");
    }
}
