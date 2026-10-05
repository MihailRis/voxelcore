#include "io.hpp"

#include <map>
#include <stdint.h>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>

#include "coders/commons.hpp"
#include "coders/gzip.hpp"
#include "coders/json.hpp"
#include "coders/toml.hpp"
#include "util/stringutil.hpp"

#include "devices/Device.hpp"

namespace fs = std::filesystem;

static std::map<std::string, std::shared_ptr<io::Device>> devices;

void io::set_device(const std::string& name, std::shared_ptr<io::Device> device) {
    devices[name] = device;
}

void io::remove_device(const std::string& name) {
    devices.erase(name);
}

std::shared_ptr<io::Device> io::get_device(const std::string& name) {
    const auto& found = devices.find(name);
    if (found == devices.end()) {
        return nullptr;
    }
    return found->second;
}

io::Device& io::require_device(const std::string& name) {
    auto device = get_device(name);
    if (!device) {
        throw std::runtime_error("io-device not found: " + name);
    }
    return *device;
}

void io::create_subdevice(
    const std::string& name, const std::string& parent, const io::path& root
) {
    auto parentDevice = get_device(parent);
    if (!parentDevice) {
        throw std::runtime_error("parent device not found for entry-point: " + parent);
    }
    set_device(name, std::make_shared<io::SubDevice>(parentDevice, root.pathPart()));
}

io::directory_iterator::directory_iterator(const io::path& folder)
    : folder(folder) {
    auto& device = io::require_device(folder.entryPoint());
    generator = device.list(folder.pathPart());
}

io::rafile::rafile(const io::path& filename)
    : file(std::make_unique<std::ifstream>(io::resolve(filename), std::ios::binary | std::ios::ate)) {
    if (!*file) {
        throw std::runtime_error("could not to open file " + filename.string());
    }
    filelength = file->tellg();
    file->seekg(0);
}

io::rafile::rafile(std::unique_ptr<std::istream> file, size_t length)
    : file(std::move(file)), filelength(length) {
}

size_t io::rafile::length() const {
    return filelength;
}

void io::rafile::seekg(std::streampos pos) {
    file->seekg(pos);
}

void io::rafile::read(char* buffer, std::streamsize size) {
    file->read(buffer, size);
}

bool io::write_bytes(
    const io::path& filename, const ubyte* data, size_t size
) {
    auto device = io::get_device(filename.entryPoint());
    if (device == nullptr) {
        throw std::runtime_error("io-device not found: " + filename.entryPoint());
    }
    auto stream = device->write(filename.pathPart());
    stream->write(reinterpret_cast<const char*>(data), size);
    return stream->good();
}

bool io::read(const io::path& filename, char* data, size_t size) {
    auto device = io::get_device(filename.entryPoint());
    if (device == nullptr) {
        return false;
    }
    auto stream = io::read(filename);
    stream->read(data, size);
    return stream->good();
}

std::unique_ptr<std::ostream> io::write(const io::path& file) {
    auto device = io::get_device(file.entryPoint());
    if (device == nullptr) {
        throw std::runtime_error("io-device not found: " + file.entryPoint());
    }
    return device->write(file.pathPart());
}

std::unique_ptr<std::istream> io::read(const io::path& filename) {
    auto device = io::get_device(filename.entryPoint());
    if (device == nullptr) {
        throw std::runtime_error("io-device not found: " + filename.entryPoint());
    }
    return device->read(filename.pathPart());
}

util::Buffer<ubyte> io::read_bytes_buffer(const path& file) {
    size_t size;
    auto bytes = io::read_bytes(file, size);
    return util::Buffer<ubyte>(std::move(bytes), size);
}

std::unique_ptr<ubyte[]> io::read_bytes(
    const io::path& filename, size_t& length
) {
    auto& device = io::require_device(filename.entryPoint());
    length = device.size(filename.pathPart());
    auto data = std::make_unique<ubyte[]>(length);
    auto stream = io::read(filename);
    stream->read(reinterpret_cast<char*>(data.get()), length);
    return stream->good() ? std::move(data) : nullptr;
}

std::vector<ubyte> io::read_bytes(const path& filename) {
    auto& device = io::require_device(filename.entryPoint());
    size_t length = device.size(filename.pathPart());
    std::vector<ubyte> data(length);
    auto stream = io::read(filename);
    stream->read(reinterpret_cast<char*>(data.data()), length);
    return data;
}

std::string io::read_string(const path& filename) {
    size_t size;
    auto bytes = read_bytes(filename, size);
    return std::string((const char*)bytes.get(), size);
}

bool io::write_string(const io::path& file, std::string_view content) {
    return io::write_bytes(file, (const ubyte*)content.data(), content.size());
}

bool io::write_json(
    const io::path& file, const dv::value& obj, bool nice
) {
    return io::write_string(file, json::stringify(obj, nice, "  "));
}

bool io::write_binary_json(
    const io::path& file, const dv::value& obj, bool compression
) {
    auto bytes = json::to_binary(obj, compression);
    return io::write_bytes(file, bytes.data(), bytes.size());
}

dv::value io::read_json(const path& filename) {
    auto text = io::read_string(filename);
    try {
        return json::parse(filename.string(), text);
    } catch (const parsing_error& err) {
        throw err.toRuntimeError();
    }
}

dv::value io::read_binary_json(const path& file) {
    size_t size;
    auto bytes = io::read_bytes(file, size);
    return json::from_binary(bytes.get(), size);
}

dv::value io::read_toml(const path& file) {
    auto text = io::read_string(file);
    try {
        return toml::parse(file.string(), text);
    } catch (const parsing_error& err) {
        throw err.toRuntimeError();
    }
}

std::vector<std::string> io::read_list(const io::path& filename) {
    auto stream = io::read(filename);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(*stream, line)) {
        util::trim(line);
        if (line.length() == 0) continue;
        if (line[0] == '#') continue;
        lines.push_back(line);
    }
    return lines;
}

bool io::is_regular_file(const io::path& file) {
    if (file.emptyOrInvalid()) {
        return false;
    }
    auto device = io::get_device(file.entryPoint());
    if (device == nullptr) {
        return false;
    }
    return device->isfile(file.pathPart());
}

bool io::is_directory(const io::path& file) {
    if (file.emptyOrInvalid()) {
        return false;
    }
    auto device = io::get_device(file.entryPoint());
    if (device == nullptr) {
        return false;
    }
    return device->isdir(file.pathPart());
}

bool io::exists(const io::path& file) {
    if (file.emptyOrInvalid()) {
        return false;
    }
    auto device = io::get_device(file.entryPoint());
    if (device == nullptr) {
        return false;
    }
    return device->exists(file.pathPart());
}

bool io::create_directory(const io::path& file) {
    auto& device = io::require_device(file.entryPoint());
    if (device.isdir(file.pathPart())) {
        return false;
    }
    return device.mkdirs(file.pathPart());
}


bool io::create_directories(const io::path& file) {
    auto& device = io::require_device(file.entryPoint());
    if (device.isdir(file.pathPart())) {
        return false;
    }
    return device.mkdirs(file.pathPart());
}

bool io::remove(const io::path& file) {
    auto& device = io::require_device(file.entryPoint());
    return device.remove(file.pathPart());
}

uint64_t io::remove_all(const io::path& file) {
    auto& device = io::require_device(file.entryPoint());
    return device.removeAll(file.pathPart());
}

bool io::copy(const io::path& src, const io::path& dst) {
    auto& srcDevice = io::require_device(src.entryPoint());
    auto& dstDevice = io::require_device(dst.entryPoint());
    if (!srcDevice.isfile(src.pathPart())) {
        return false;
    }
    auto input = srcDevice.read(src.pathPart());
    auto output = dstDevice.write(dst.pathPart());
    size_t size = srcDevice.size(src.pathPart());
    std::vector<char> buffer(16'384);
    while (size > 0) {
        size_t read = std::min(size, buffer.size());
        input->read(buffer.data(), read);
        auto gcount = input->gcount();
        output->write(buffer.data(), gcount);
        size -= gcount;
        if (input->eof()) {
            break;
        }
        if (!input->good() || !output->good()) {
            return false;
        }
    }
    return output->good();
}

uint64_t io::copy_all(const io::path& src, const io::path& dst) {
    auto& srcDevice = io::require_device(src.entryPoint());
    auto& dstDevice = io::require_device(dst.entryPoint());
    auto dstPath = dst.pathPart();
    if (!dstDevice.isdir(dstPath) && !dstDevice.mkdirs(dstPath)) {
        return 0;
    }
    uint64_t count = 0;
    for (auto& srcSubFile : directory_iterator(src)) {
        auto dstSubFile = dst / srcSubFile.name();
        auto srcSubPath = srcSubFile.pathPart();
        auto dstSubPath = dstSubFile.pathPart();
        if (srcDevice.isdir(srcSubPath)) {
            if (!dstDevice.mkdirs(dstSubPath)) {
                continue;
            }
            count += copy_all(srcSubFile, dstSubFile);
        } else if (copy(srcSubFile, dstSubFile)) {
            count++;
        }
    }
    return count;
}

size_t io::file_size(const io::path& file) {
    auto& device = io::require_device(file.entryPoint());
    return device.size(file.pathPart());
}

io::file_time_type io::last_write_time(const io::path& file) {
    auto& device = io::require_device(file.entryPoint());
    return device.lastWriteTime(file.pathPart());
}

std::filesystem::path io::resolve(const io::path& file) {
    auto device = io::get_device(file.entryPoint());
    if (device == nullptr) {
        return {};
    }
    return device->resolve(file.pathPart());
}

#include <map>

#include "coders/json.hpp"
#include "coders/toml.hpp"

using DecodeFunc = dv::value(*)(std::string_view, std::string_view);

static std::map<fs::path, DecodeFunc> data_decoders {
    {fs::u8path(".json"), json::parse},
    {fs::u8path(".toml"), toml::parse},
};

bool io::is_data_file(const io::path& file) {
    return is_data_interchange_format(file.extension());
}

bool io::is_data_interchange_format(const std::string& ext) {
    return data_decoders.find(ext) != data_decoders.end();
}

dv::value io::read_object(const path& file) {
    const auto& found = data_decoders.find(file.extension());
    if (found == data_decoders.end()) {
        throw std::runtime_error("unknown file format");
    }
    auto text = read_string(file);
    try {
        return found->second(file.string(), text);
    } catch (const parsing_error& err) {
        throw std::runtime_error(err.errorLog());
    }
}
