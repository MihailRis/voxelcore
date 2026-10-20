#include "ogg.hpp"

#include <vorbis/codec.h>
#include <vorbis/vorbisfile.h>

#include <string>

#include "io/io.hpp"
#include "audio/audio.hpp"
#include "debug/Logger.hpp"
#include "typedefs.hpp"

static debug::Logger logger("ogg");

namespace fs = std::filesystem;
using namespace audio;

static inline std::string vorbis_error_message(int code) {
    switch (code) {
        case 0:
            return "no error";
        case OV_EREAD:
            return "a read from media returned an error";
        case OV_ENOTVORBIS:
            return "the given file/data was not recognized as Ogg Vorbis data";
        case OV_EVERSION:
            return "vorbis version mismatch";
        case OV_EBADHEADER:
            return "invalid Vorbis bitstream header";
        case OV_EFAULT:
            return "internal logic fault";
        case OV_EINVAL:
            return "invalid read operation";
        case OV_EBADLINK:
            return "the given link exists in the Vorbis data stream,"
                   " but is not decipherable due to garbacge or corruption";
        case OV_ENOSEEK:
            return "the given stream is not seekable";
        case OV_EIMPL:
            return "feature not implemented";
        default:
            return "unknown error [" + std::to_string(code) + "]";
    }
}

std::unique_ptr<audio::PCM> ogg::load_pcm(
    const io::path& file, bool headerOnly
) {
    OggVorbis_File vf;
    FILE* fileHandle = nullptr;
#ifdef _WIN32
    if (errno_t err =
            _wfopen_s(
                &fileHandle, io::resolve(file).wstring().c_str(), L"rb"
            ) != 0) {
#else
    if ((fileHandle = fopen(io::resolve(file).u8string().c_str(), "rb")) ==
        nullptr) {
#endif
        throw std::runtime_error(
            "could not open file " + file.string() + " for reading"
        );
    }

    if (int code = ov_open(fileHandle, &vf, nullptr, 0)) {
        if (fileHandle) {
            fclose(fileHandle);
        }
        throw std::runtime_error("vorbis: " + vorbis_error_message(code));
    }
    std::vector<char> data;

    vorbis_info* info = ov_info(&vf, -1);
    uint channels = info->channels;
    uint sampleRate = info->rate;
    bool seekable = ov_seekable(&vf);
    size_t totalSamples = seekable ? ov_pcm_total(&vf, -1) : 0;

    if (!headerOnly) {
        const int bufferSize = 4096;
        int section = 0;
        char buffer[bufferSize];

        bool eof = false;
        while (!eof) {
            long ret = ov_read(&vf, buffer, bufferSize, 0, 2, true, &section);
            if (ret == 0) {
                eof = true;
            } else if (ret < 0) {
                logger.error()
                    << "ogg::load_pcm: " << vorbis_error_message(ret);
            } else {
                data.insert(
                    data.end(), std::begin(buffer), std::begin(buffer) + ret
                );
            }
        }
        totalSamples = data.size() / channels / 2;
    }
    ov_clear(&vf);
    return std::make_unique<PCM>(
        std::move(data), totalSamples, channels, 16, sampleRate, seekable
    );
}

class OggStream : public PCMStream {
    OggVorbis_File vf;
    bool closed = false;
    uint channels;
    uint sampleRate;
    size_t totalSamples = 0;
    bool seekable;
public:
    OggStream(OggVorbis_File vf) : vf(vf) {
        vorbis_info* info = ov_info(&vf, -1);
        channels = info->channels;
        sampleRate = info->rate;
        seekable = ov_seekable(&vf);
        if (seekable) {
            totalSamples = ov_pcm_total(&vf, -1);
        }
    }

    ~OggStream() {
        if (!closed) {
            close();
        }
    }

    size_t read(char* buffer, size_t bufferSize) override {
        if (closed) {
            return 0;
        }
        int bitstream = 0;
        long bytes = ov_read(&vf, buffer, bufferSize, 0, 2, true, &bitstream);
        if (bytes < 0) {
            logger.error() << "ogg::load_pcm: " << vorbis_error_message(bytes)
                           << " " << bytes;
            return PCMStream::ERROR;
        }
        return bytes;
    }

    void close() override {
        if (!closed) {
            ov_clear(&vf);
            closed = true;
        }
    }

    bool isOpen() const override {
        return !closed;
    }

    size_t getTotalSamples() const override {
        return totalSamples;
    }

    duration_t getTotalDuration() const override {
        return static_cast<duration_t>(totalSamples) /
               static_cast<duration_t>(sampleRate);
    }

    uint getChannels() const override {
        return channels;
    }

    uint getSampleRate() const override {
        return sampleRate;
    }

    uint getBitsPerSample() const override {
        return 16;
    }

    bool isSeekable() const override {
        return seekable;
    }

    void seek(size_t position) override {
        if (!closed && seekable) {
            ov_pcm_seek(&vf, position);
        }
    }
};

std::unique_ptr<PCMStream> ogg::create_stream(const io::path& file) {
    OggVorbis_File vf;
    FILE* fileHandle = nullptr;
#ifdef _WIN32
    if (errno_t err =
            _wfopen_s(
                &fileHandle, io::resolve(file).wstring().c_str(), L"rb"
            ) != 0) {
#else
    if ((fileHandle = fopen(io::resolve(file).u8string().c_str(), "rb")) ==
        nullptr) {
#endif
        throw std::runtime_error(
            "could not open file " + file.string() + " for reading"
        );
    }

    if (int code = ov_open(fileHandle, &vf, nullptr, 0)) {
        if (fileHandle) {
            fclose(fileHandle);
        }
        throw std::runtime_error("vorbis: " + vorbis_error_message(code));
    }
    return std::make_unique<OggStream>(vf);
}
