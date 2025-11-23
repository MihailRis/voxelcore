#pragma once

#include <memory>
#include <string>

#include "io/fwd.hpp"
#include "util/Buffer.hpp"
#include "util/EnumMetadata.hpp"
#include "util/span.hpp"
#include "typedefs.hpp"

class ImageData;

namespace imageio {
    enum class ImageFileFormat {
        PNG
    };

    VC_ENUM_METADATA(ImageFileFormat)
        {"png", ImageFileFormat::PNG},
    VC_ENUM_END

    inline const std::string PNG = ".png";

    bool is_read_supported(const std::string& extension);
    bool is_write_supported(const std::string& extension);

    std::unique_ptr<ImageData> read(const io::path& file);
    void write(const io::path& file, const ImageData* image);
    std::unique_ptr<ImageData> decode(ImageFileFormat format, util::span<ubyte> src);
    util::Buffer<unsigned char> encode(ImageFileFormat format, const ImageData& image);
}
