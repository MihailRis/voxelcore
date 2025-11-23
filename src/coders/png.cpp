#include "png.hpp"

#include <png.h>

#include <iostream>

#include "debug/Logger.hpp"
#include "io/io.hpp"
#include "graphics/core/Texture.hpp"
#include "graphics/core/ImageData.hpp"

static debug::Logger logger("png-coder");

static util::Buffer<ubyte> write_to_memory(uint width, uint height, const ubyte* data, bool alpha) {
    uint pixsize = alpha ? 4 : 3;

    std::vector<ubyte> buffer;
    png_structp png_ptr = png_create_write_struct(
        PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr
    );
    png_infop info_ptr = png_create_info_struct(png_ptr);

    png_set_write_fn(
        png_ptr,
        &buffer,
        [](png_structp pngPtr, png_bytep data, png_size_t length) {
            auto& buf = *reinterpret_cast<std::vector<ubyte>*>(png_get_io_ptr(pngPtr));
            buf.insert(
                buf.end(),
                reinterpret_cast<ubyte*>(data),
                reinterpret_cast<ubyte*>(data) + length
            );
        },
        nullptr
    );

    png_set_IHDR(
        png_ptr,
        info_ptr,
        width,
        height,
        8,
        alpha ? PNG_COLOR_TYPE_RGBA : PNG_COLOR_TYPE_RGB,
        PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_BASE,
        PNG_FILTER_TYPE_BASE
    );

    png_write_info(png_ptr, info_ptr);

    auto row = std::make_unique<png_byte[]>(pixsize * width);
    for (uint y = 0; y < height; y++) {
        for (uint x = 0; x < width; x++) {
            for (uint i = 0; i < pixsize; i++) {
                row[x * pixsize + i] =
                    (png_byte)data[(y * width + x) * pixsize + i];
            }
        }
        png_write_row(png_ptr, row.get());
    }

    png_write_end(png_ptr, nullptr);
    png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    return util::Buffer<ubyte>(buffer.data(), buffer.size());
}

// returns 0 if all-right, 1 otherwise
static int png_write(
    const char* filename, uint width, uint height, const ubyte* data, bool alpha
) {
    uint pixsize = alpha ? 4 : 3;

    // Open file for writing (binary mode)
    FILE* fp = fopen(filename, "wb");
    if (fp == nullptr) {
        logger.error() << "could not open file " << filename << " for writing";
        return 1;
    }

    // Initialize write structure
    png_structp png_ptr = png_create_write_struct(
        PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr
    );
    if (png_ptr == nullptr) {
        logger.error() << "could not allocate write struct";
        fclose(fp);
        return 1;
    }

    // Initialize info structure
    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == nullptr) {
        logger.error() << "could not allocate info struct";
        fclose(fp);
        png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
        png_destroy_write_struct(&png_ptr, (png_infopp) nullptr);
        return 1;
    }

    // Setup Exception handling
    if (setjmp(png_jmpbuf(png_ptr))) {
        logger.error() << "error during png creation";
        fclose(fp);
        png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
        png_destroy_write_struct(&png_ptr, &info_ptr);
        return 1;
    }

    png_init_io(png_ptr, fp);

    // Write header (8 bit colour depth)
    png_set_IHDR(
        png_ptr,
        info_ptr,
        width,
        height,
        8,
        alpha ? PNG_COLOR_TYPE_RGBA : PNG_COLOR_TYPE_RGB,
        PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_BASE,
        PNG_FILTER_TYPE_BASE
    );

    png_write_info(png_ptr, info_ptr);

    auto row = std::make_unique<png_byte[]>(pixsize * width);
    // Write image data
    for (uint y = 0; y < height; y++) {
        for (uint x = 0; x < width; x++) {
            for (uint i = 0; i < pixsize; i++) {
                row[x * pixsize + i] =
                    (png_byte)data[(y * width + x) * pixsize + i];
            }
        }
        png_write_row(png_ptr, row.get());
    }

    // End write
    png_write_end(png_ptr, nullptr);

    fclose(fp);
    png_free_data(png_ptr, info_ptr, PNG_FREE_ALL, -1);
    png_destroy_write_struct(&png_ptr, &info_ptr);
    return 0;
}

struct InMemoryReader {
    const ubyte* bytes;
    size_t size;
    size_t offset;
};

static void read_in_memory(png_structp pngPtr, png_bytep dst, png_size_t toread) {
    png_voidp ioPtr = png_get_io_ptr(pngPtr);
    if (ioPtr == nullptr) {
        throw std::runtime_error("png_get_io_ptr(...) -> NULL");
    }
    auto& reader = *reinterpret_cast<InMemoryReader*>(ioPtr);
    if (reader.offset + toread > reader.size) {
        throw std::runtime_error("buffer underflow");
    }
    std::memcpy(dst, reader.bytes + reader.offset, toread);
    reader.offset += toread;
}

std::unique_ptr<ImageData> png::load_image(const ubyte* bytes, size_t size) {
    if (size < 8 || !png_check_sig(bytes, 8)) {
        throw std::runtime_error("invalid png signature");
    }
    png_structp pngPtr = nullptr;
    pngPtr = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (pngPtr == nullptr) {
        throw std::runtime_error("failed png_create_read_struct");
    }
    png_infop infoPtr = nullptr;
    infoPtr = png_create_info_struct(pngPtr);
    if(infoPtr == nullptr) {
        png_destroy_read_struct(&pngPtr, nullptr, nullptr);
        throw std::runtime_error("failed png_create_info_struct");
    }

    InMemoryReader reader {bytes, size, 0};

    png_set_read_fn(pngPtr, &reader, read_in_memory);
    png_read_info(pngPtr, infoPtr);

    png_uint_32 width = 0;
    png_uint_32 height = 0;
    int bitDepth = 0;
    int colorType = -1;
    png_uint_32 retval = png_get_IHDR(pngPtr, infoPtr,
        &width,
        &height,
        &bitDepth,
        &colorType,
        nullptr, nullptr, nullptr
    );
    if (retval != 1) {
        png_destroy_read_struct(&pngPtr, &infoPtr, nullptr);
        throw std::runtime_error("failed png_get_IHDR");
    }
    if (bitDepth == 16) png_set_strip_16(pngPtr);

    if (colorType == PNG_COLOR_TYPE_PALETTE) {
        png_set_palette_to_rgb(pngPtr);
    }
    if (colorType == PNG_COLOR_TYPE_GRAY && bitDepth < 8) {
        png_set_expand_gray_1_2_4_to_8(pngPtr);
    }
    if (png_get_valid(pngPtr, infoPtr, PNG_INFO_tRNS)) {
        png_set_tRNS_to_alpha(pngPtr);
    }
    // These color_type don't have an alpha channel then fill it with 0xff.
    if (colorType == PNG_COLOR_TYPE_RGB || colorType == PNG_COLOR_TYPE_GRAY ||
        colorType == PNG_COLOR_TYPE_PALETTE) {
        png_set_filler(pngPtr, 0xFF, PNG_FILLER_AFTER);
    }
    if (colorType == PNG_COLOR_TYPE_GRAY ||
        colorType == PNG_COLOR_TYPE_GRAY_ALPHA) {
        png_set_gray_to_rgb(pngPtr);
    }
    png_read_update_info(pngPtr, infoPtr);

    int rowBytes = png_get_rowbytes(pngPtr, infoPtr);

    //  png_get_color_type returns 2 (RGB) but raster always have alpha channel
    //  due to PNG_FILLER_AFTER
    colorType = 6;
    bitDepth = png_get_bit_depth(pngPtr, infoPtr);

    auto imageData = std::make_unique<png_byte[]>(rowBytes * height);
    auto rowPointers = std::make_unique<png_byte*[]>(height);
    for (int i = 0; i < height; ++i) {
        rowPointers[height - 1 - i] = imageData.get() + i * rowBytes;
    }
    png_read_image(pngPtr, rowPointers.get());

    ImageFormat format = ImageFormat::rgba8888;
    switch (colorType) {
        case PNG_COLOR_TYPE_RGBA:
            format = ImageFormat::rgba8888;
            break;
        case PNG_COLOR_TYPE_RGB:
            format = ImageFormat::rgb888;
            break;
        default:
            png_destroy_read_struct(&pngPtr, &infoPtr, nullptr);
            throw std::runtime_error(
                "color type " + std::to_string(colorType) + " is not supported!"
            );
    }
    auto image = std::make_unique<ImageData>(
        format, width, height, std::move(imageData)
    );
    png_destroy_read_struct(&pngPtr, &infoPtr, nullptr);
    return image;
}

std::unique_ptr<Texture> png::load_texture(const ubyte* bytes, size_t size) {
    auto image = load_image(bytes, size);
    auto texture = Texture::from(image.get());
    texture->setNearestFilter();
    return texture;
}

std::unique_ptr<Texture> png::load_texture(const std::string& filename) {
    auto bytes = io::read_bytes_buffer(filename);
    try {
        return load_texture(bytes.data(), bytes.size());
    } catch (const std::runtime_error& err) {
        throw std::runtime_error(
            "could not to load " + filename + ": " + err.what()
        );
    }
}

void png::write_image(const std::string& filename, const ImageData* image) {
    png_write(
        filename.c_str(),
        image->getWidth(),
        image->getHeight(),
        (const ubyte*)image->getData(),
        image->getFormat() == ImageFormat::rgba8888
    );
}

util::Buffer<ubyte> png::encode_image(const ImageData& image) {
    auto format = image.getFormat();
    return write_to_memory(
        image.getWidth(),
        image.getHeight(),
        image.getData(),
        format == ImageFormat::rgba8888
    );
}
