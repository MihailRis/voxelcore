#include "ZipFileDevice.hpp"

#include <vector>

#include "debug/Logger.hpp"
#include "io/memory_istream.hpp"
#include "io/memory_ostream.hpp"
#include "io/deflate_istream.hpp"
#include "io/deflate_ostream.hpp"
#include "util/data_io.hpp"
#include "util/Buffer.hpp"

static debug::Logger logger("zip-file");

using namespace io;
using namespace std::chrono;

static constexpr uint32_t EOCD_SIGNATURE = 0x06054b50;
static constexpr uint32_t CENTRAL_DIR_SIGNATURE = 0x02014b50;
static constexpr uint32_t LOCAL_FILE_SIGNATURE = 0x04034b50;
static constexpr uint32_t COMPRESSION_NONE = 0;
static constexpr uint32_t COMPRESSION_DEFLATE = 8;

namespace {
    template<typename T>
    T read_int(std::unique_ptr<std::istream>& file) {
        T value = 0;
        file->read(reinterpret_cast<char*>(&value), sizeof(value));
        return dataio::le2h(value);
    }

    template<typename T>
    void read_int(std::unique_ptr<std::istream>& file, T& value) {
        file->read(reinterpret_cast<char*>(&value), sizeof(value));
        value = dataio::le2h(value);
    }
    file_time_type msdos_to_file_time(uint16_t date, uint16_t time) {
        uint16_t year = ((date >> 9) & 0x7F) + 1980;
        uint16_t month = (date >> 5) & 0x0F;
        uint16_t day = date & 0x1F;

        uint16_t hours = (time >> 11) & 0x1F;
        uint16_t minutes = (time >> 5) & 0x3F;
        uint16_t seconds = (time & 0x1F) * 2;

        std::tm time_struct = {};
        time_struct.tm_year = year - 1900;
        time_struct.tm_mon = month - 1;
        time_struct.tm_mday = day;
        time_struct.tm_hour = hours;
        time_struct.tm_min = minutes;
        time_struct.tm_sec = seconds;
        time_struct.tm_isdst = -1;

        std::time_t time_t_value = std::mktime(&time_struct);
        auto time_point = system_clock::from_time_t(time_t_value);
        return file_time_type::clock::now() + (time_point - system_clock::now());
    }

    uint32_t to_ms_dos_timestamp(const file_time_type& fileTime) {
        auto timePoint = time_point_cast<system_clock::duration>(
            fileTime - file_time_type::clock::now() + system_clock::now()
        );
        std::time_t timeT = system_clock::to_time_t(timePoint);
        std::tm tm = *std::localtime(&timeT);
        uint16_t date = (tm.tm_year - 80) << 9 | (tm.tm_mon + 1) << 5 | tm.tm_mday;
        uint16_t time = (tm.tm_hour << 11) | (tm.tm_min << 5) | (tm.tm_sec / 2);
        return (date << 16) | time;
    }
}

ZipFileDevice::Entry ZipFileDevice::readEntry() {
    // Read entry info
    Entry entry {};
    read_int(file, entry.versionMadeBy);
    read_int(file, entry.versionNeeded);
    read_int(file, entry.flags);
    read_int(file, entry.compressionMethod);
    read_int(file, entry.modTime);
    read_int(file, entry.modDate);
    read_int(file, entry.crc32);
    read_int(file, entry.compressedSize);
    read_int(file, entry.uncompressedSize);
    auto filename_len = read_int<uint16_t>(file);
    auto extra_field_len = read_int<uint16_t>(file);
    auto file_comment_len = read_int<uint16_t>(file);
    read_int(file, entry.diskNumberStart);
    read_int(file, entry.internalAttributes);
    read_int(file, entry.externalAttributes);
    read_int(file, entry.localHeaderOffset);

    entry.fileName.resize(filename_len, '\0');
    file->read(entry.fileName.data(), filename_len);

    // Skip extra field and file comment
    file->seekg(extra_field_len + file_comment_len, std::ios::cur);

    if (entry.diskNumberStart == 0xFF) {
        throw std::runtime_error("zip64 is not supported");
    }

    for (size_t i = 0; i < entry.fileName.length(); i++) {
        if (entry.fileName[i] == '\\') {
            entry.fileName[i] = '/';
        }
    }
    if (entry.fileName[entry.fileName.length() - 1] == '/') {
        entry.isDirectory = true;
        entry.fileName = entry.fileName.substr(0, entry.fileName.length() - 1);
    }
    return entry;
}

void ZipFileDevice::findBlob(Entry& entry) {
    file->seekg(entry.localHeaderOffset);
    if (read_int<uint32_t>(file) != LOCAL_FILE_SIGNATURE) {
        throw std::runtime_error("invalid local file signature");
    }
    read_int<uint16_t>(file); // version
    read_int<uint16_t>(file); // flags
    read_int<uint16_t>(file); // compression method
    read_int<uint16_t>(file); // last modification time
    read_int<uint16_t>(file); // last modification date
    read_int<uint32_t>(file); // crc32
    read_int<uint32_t>(file); // compressed size
    read_int<uint32_t>(file); // uncompressed size
    auto name_len = read_int<uint16_t>(file);
    auto extra_field_len = read_int<uint16_t>(file);

    // Skip extra field and file comment
    file->seekg(name_len + extra_field_len, std::ios::cur);
    entry.blobOffset = file->tellg();
}

ZipFileDevice::ZipFileDevice(
    std::unique_ptr<std::istream> filePtr, FileSeparateFunc separateFunc
)
    : file(std::move(filePtr)), separateFunc(std::move(separateFunc)) {
    // Searching for EOCD
    file->seekg(0, std::ios::end);
    std::streampos file_size = file->tellg();

    bool foundEOCD = false;
    for (int pos = static_cast<int>(file_size)-4; pos >= 0; --pos) {
        file->seekg(pos);
        if (read_int<uint32_t>(file) == EOCD_SIGNATURE) {
            foundEOCD = true;
            break;
        }
    }
    if (!foundEOCD) {
        throw std::runtime_error("EOCD not found, ZIP file is invalid");
    }

    // Reading EOCD
    read_int<uint16_t>(file); // diskNumber
    read_int<uint16_t>(file); // centralDirDisk
    read_int<uint16_t>(file); // numEntriesThisDisk
    auto total_entries = read_int<uint16_t>(file);
    read_int<uint32_t>(file); // centralDirSize
    auto central_dir_offset = read_int<uint32_t>(file);
    read_int<uint16_t>(file); // commentLength

    file->seekg(central_dir_offset);

    for (uint16_t i = 0; i < total_entries; i++) {
        if (read_int<uint32_t>(file) != CENTRAL_DIR_SIGNATURE) {
            logger.error() << "invalid central directory entry";
            break;
        }
        // Read entry info
        Entry entry = readEntry();
        entries[entry.fileName] = std::move(entry);
    }

    for (auto& [name, _] : entries) {
        io::path path = name;

        while (!(path = path.parent()).pathPart().empty()) {
            if (entries.find(path.pathPart()) != entries.end()) {
                continue;
            }
            Entry entry {};
            entry.isDirectory = true;
            entries[path.pathPart()] = entry;
        }
        break;
    }

    for (auto& [_, entry] : entries) {
        findBlob(entry);
    }
}

std::filesystem::path ZipFileDevice::resolve(std::string_view path) {
    throw std::runtime_error("unable to resolve filesystem path");
}

std::unique_ptr<std::ostream> ZipFileDevice::write(std::string_view path) {
    return nullptr;
}

std::unique_ptr<std::istream> ZipFileDevice::read(std::string_view path) {
    const auto& found = entries.find(std::string(path));
    if (found == entries.end()) {
        throw std::runtime_error("could not to open file zip://" + std::string(path));
    }
    auto& entry = found->second;
    if (entry.isDirectory) {
        throw std::runtime_error("zip://" + std::string(path) + " is directory");
    }
    if (entry.blobOffset == 0) {
        findBlob(entry);
    }
    std::unique_ptr<std::istream> src_stream;
    if (separateFunc) {
        // Create new istream for concurrent data reading
        src_stream = separateFunc();
        src_stream->seekg(entry.blobOffset);
    } else {
        // Read compressed data to memory if istream cannot be separated
        file->seekg(entry.blobOffset);
        util::Buffer<char> buffer(entry.compressedSize);
        file->read(buffer.data(), buffer.size());
        src_stream = std::make_unique<memory_istream>(std::move(buffer));
    }
    if (entry.compressionMethod == COMPRESSION_NONE) {
        return src_stream;
    } else if (entry.compressionMethod == COMPRESSION_DEFLATE) {
        return std::make_unique<deflate_istream>(std::move(src_stream));
    } else {
        throw std::runtime_error(
            "unsupported compression method [" +
            std::to_string(entry.compressionMethod) + "]"
        );
    }
}

size_t ZipFileDevice::size(std::string_view path) {
    const auto& found = entries.find(std::string(path));
    if (found == entries.end()) {
        return 0;
    }
    return found->second.uncompressedSize;
}

file_time_type ZipFileDevice::lastWriteTime(std::string_view path) {
    const auto& found = entries.find(std::string(path));
    if (found == entries.end()) {
        return file_time_type::min();
    }
    return msdos_to_file_time(found->second.modDate, found->second.modTime);
}

bool ZipFileDevice::exists(std::string_view path) {
    return entries.find(std::string(path)) != entries.end();
}

bool ZipFileDevice::isdir(std::string_view path) {
    if (path.empty()) {
        return true;
    }
    const auto& found = entries.find(std::string(path));
    if (found == entries.end()) {
        return false;
    }
    return found->second.isDirectory;
}

bool ZipFileDevice::isfile(std::string_view path) {
    const auto& found = entries.find(std::string(path));
    if (found == entries.end()) {
        return false;
    }
    return !found->second.isDirectory;
}

bool ZipFileDevice::mkdir(std::string_view path) {
    return false;
}

bool ZipFileDevice::mkdirs(std::string_view path) {
    return false;
}

bool ZipFileDevice::remove(std::string_view path) {
    return false;
}

uint64_t ZipFileDevice::removeAll(std::string_view path) {
    return 0;
}

class ListPathsGenerator : public PathsGenerator {
public:
    ListPathsGenerator(std::vector<std::string> names)
        : names(std::move(names)) {};

    bool next(path& dst) override {
        if (current == names.size()) {
            return false;
        }
        dst = names[current++];
        return true;
    }
private:
    std::vector<std::string> names;
    size_t current = 0;
};

std::unique_ptr<PathsGenerator> ZipFileDevice::list(std::string_view path) {
    std::vector<std::string> names;
    if (path.empty()) {
        for (const auto& [name, entry] : entries) {
            if (name.find('/') == std::string::npos) {
                names.push_back(name);
            }
        }
    } else {
        auto folder = std::string(path) + "/";
        size_t folder_len = folder.length();
        for (const auto& [name, entry] : entries) {
            if (name.find(folder) != 0) {
                continue;
            }
            size_t pos = name.find('/', folder_len);
            if (pos == std::string::npos) {
                names.push_back(name.substr(folder_len, pos - folder_len));
            }
            if (pos == name.length() - 1) {
                names.push_back(name.substr(folder_len, pos - folder_len));
            }
        }
    }
    return std::make_unique<ListPathsGenerator>(std::move(names));
}

#include "io/io.hpp"
#include "coders/byte_utils.hpp"

static void write_headers(
    std::ostream& file,
    const std::string& name,
    size_t source_Size,
    size_t compressed_size,
    uint32_t crc,
    int compression_method,
    const file_time_type& last_write_time,
    ByteBuilder& central_dir
) {
    auto timestamp = to_ms_dos_timestamp(last_write_time);
    ByteBuilder header;
    header.putInt32(LOCAL_FILE_SIGNATURE);
    header.putInt16(10); // version
    header.putInt16(0); // flags
    header.putInt16(compression_method); // compression method
    header.putInt32(timestamp); // last modification datetime
    header.putInt32(crc); // crc32
    header.putInt32(compressed_size);
    header.putInt32(source_Size);
    header.putInt16(name.length());
    header.putInt16(0); // extra field length
    header.put(reinterpret_cast<const ubyte*>(name.data()), name.length());

    size_t local_header_offset = file.tellp();
    file.write(reinterpret_cast<const char*>(header.data()), header.size());

    central_dir.putInt32(CENTRAL_DIR_SIGNATURE);
    central_dir.putInt16(10); // version
    central_dir.putInt16(0); // version
    central_dir.putInt16(0); // flags
    central_dir.putInt16(compression_method); // compression method
    central_dir.putInt32(timestamp); // last modification datetime
    central_dir.putInt32(crc); // crc32
    central_dir.putInt32(compressed_size);
    central_dir.putInt32(source_Size);
    central_dir.putInt16(name.length());
    central_dir.putInt16(0); // extra field length
    central_dir.putInt16(0); // file comment length
    central_dir.putInt16(0); // disk number start
    central_dir.putInt16(0); // internal attributes
    central_dir.putInt32(0); // external attributes
    central_dir.putInt32(local_header_offset); // local header offset
    central_dir.put(reinterpret_cast<const ubyte*>(name.data()), name.length());
}

static size_t write_zip(
    const std::string& root,
    const path& folder,
    std::ostream& file,
    ByteBuilder& central_dir
) {
    size_t entries = 0;
    for (const auto& entry : io::directory_iterator(folder)) {
        auto name = entry.pathPart().substr(root.length());
        auto last_write_time = io::last_write_time(entry);
        if (io::is_directory(entry)) {
            name = name + "/";
            write_headers(
                file,
                name,
                0,
                0,
                0,
                COMPRESSION_NONE,
                last_write_time,
                central_dir
            );
            entries += write_zip(root, entry, file, central_dir) + 1;
        } else {
            auto uncompressed = io::read_bytes_buffer(entry);
            uint32_t crc = crc32(0, uncompressed.data(), uncompressed.size());
            memory_ostream memory_stream;
            {
                deflate_ostream deflate_stream(memory_stream);
                deflate_stream.write(
                    reinterpret_cast<char*>(uncompressed.data()),
                    uncompressed.size()
                );
                deflate_stream.flush();
            }
            auto data = memory_stream.release();
            size_t data_size = data.size();
            write_headers(
                file,
                name,
                uncompressed.size(),
                data_size,
                crc,
                COMPRESSION_DEFLATE,
                last_write_time,
                central_dir
            );
            file.write(reinterpret_cast<const char*>(data.data()), data_size);
            entries++;
        }
    }
    return entries;
}

void io::write_zip(const path& folder, const path& file) {
    ByteBuilder central_dir;
    auto out = io::write(file);
    size_t entries = write_zip(folder.pathPart(), folder, *out, central_dir);

    size_t central_dir_offset = out->tellp();
    out->write(reinterpret_cast<const char*>(central_dir.data()), central_dir.size());
    
    ByteBuilder eocd;
    eocd.putInt32(EOCD_SIGNATURE);
    eocd.putInt16(0); // disk number
    eocd.putInt16(0); // central dir disk
    eocd.putInt16(entries); // num entries
    eocd.putInt16(entries); // total entries
    eocd.putInt32(central_dir.size()); // central dir size
    eocd.putInt32(central_dir_offset); // central dir offset
    eocd.putInt16(0); // comment length
    out->write(reinterpret_cast<const char*>(eocd.data()), eocd.size());
}
