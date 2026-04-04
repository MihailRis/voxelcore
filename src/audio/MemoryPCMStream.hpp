#pragma once

#include <vector>

#include "audio.hpp"
#include "util/span.hpp"

namespace audio {
    class MemoryPCMStream : public PCMStream {
    public:
        MemoryPCMStream(uint sampleRate, uint channels, uint bitsPerSample);

        void feed(util::span<ubyte> bytes);

        bool isOpen() const override;

        void close() override;

        size_t read(char* buffer, size_t bufferSize) override;

        size_t getTotalSamples() const override;

        duration_t getTotalDuration() const override;

        uint getChannels() const override;

        uint getSampleRate() const override;

        uint getBitsPerSample() const override;

        bool isSeekable() const override;

        void seek(size_t position) override;

        size_t available() const;
    private:
        uint sampleRate;
        uint channels;
        uint bitsPerSample;
        bool open = true;

        std::vector<ubyte> buffer;
    };
}
