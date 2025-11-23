#include "MemoryPCMStream.hpp"

#include <cstring>

using namespace audio;

MemoryPCMStream::MemoryPCMStream(
    uint sampleRate, uint channels, uint bitsPerSample
)
    : sampleRate(sampleRate), channels(channels), bitsPerSample(bitsPerSample) {
}

void MemoryPCMStream::feed(util::span<ubyte> bytes) {
    buffer.insert(buffer.end(), bytes.begin(), bytes.end());
}

bool MemoryPCMStream::isOpen() const {
    return open;
}

void MemoryPCMStream::close() {
    open = false;
    buffer = {};
}

size_t MemoryPCMStream::read(char* dst, size_t bufferSize) {
    if (!open || buffer.empty()) {
        return PCMStream::ERROR;
    }
    size_t count = std::min<size_t>(bufferSize, buffer.size());
    std::memcpy(dst, buffer.data(), count);
    buffer.erase(buffer.begin(), buffer.begin() + count);
    return count;
}

size_t MemoryPCMStream::getTotalSamples() const {
    return 0;
}

duration_t MemoryPCMStream::getTotalDuration() const {
    return 0.0;
}

uint MemoryPCMStream::getChannels() const {
    return channels;
}

uint MemoryPCMStream::getSampleRate() const {
    return sampleRate;
}

uint MemoryPCMStream::getBitsPerSample() const {
    return bitsPerSample;
}

bool MemoryPCMStream::isSeekable() const {
    return false;
}

void MemoryPCMStream::seek(size_t position) {}

size_t MemoryPCMStream::available() const {
    return buffer.size();
}
