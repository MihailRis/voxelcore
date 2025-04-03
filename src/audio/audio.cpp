#include "audio.hpp"

#include <iostream>
#include <stdexcept>
#include <utility>

#include "io/io.hpp"
#include "coders/ogg.hpp"
#include "coders/wav.hpp"
#include "AL/ALAudio.hpp"
#include "NoAudio.hpp"
#include "debug/Logger.hpp"
#include "util/ObjectsKeeper.hpp"

static debug::Logger logger("audio");

using namespace audio;

namespace {
    speakerid_t nextId = 1;
    Backend* backend;
    std::unordered_map<speakerid_t, std::unique_ptr<Speaker>> speakers;
    std::unordered_map<speakerid_t, std::shared_ptr<Stream>> streams;
    std::vector<std::unique_ptr<Channel>> channels;
    util::ObjectsKeeper objects_keeper {};
}

Channel::Channel(std::string name) : name(std::move(name)) {
}

float Channel::getVolume() const {
    return volume;
}

void Channel::setVolume(float volume) {
    this->volume = std::max(0.0f, std::min(volume, 1.0f));
}

const std::string& Channel::getName() const {
    return name;
}

// see pause logic in audio::update
void Channel::pause() {
    paused = true;
}

// see pause logic in audio::update
void Channel::resume() {
    paused = false;
}

bool Channel::isPaused() const {
    return paused;
}

size_t PCMStream::readFully(char* buffer, size_t bufferSize, bool loop) {
    if (!isOpen()) {
        return 0;
    }
    size_t size = 0;
    do {
        size_t bytes = 0;
        do {
            bytes = read(buffer, bufferSize);
            if (bytes == PCMStream::ERROR) {
                return size;
            }
            size += bytes;
            bufferSize -= bytes;
            buffer += bytes;
        } while (bytes > 0 && bufferSize > 0);

        if (bufferSize == 0) {
            break;
        }

        if (loop) {
            seek(0);
        }
    } while (loop);
    return size;
}

/// @brief pcm source that does not initialize buffer
class PCMVoidSource : public PCMStream {
    size_t totalSamples;
    size_t remain;
    uint sampleRate;
    bool seekable;
    bool closed = false;
public:
    PCMVoidSource(size_t totalSamples, uint sampleRate, bool seekable)
        : totalSamples(totalSamples),
          remain(totalSamples),
          sampleRate(sampleRate),
          seekable(seekable) {
    }

    size_t read(char*, size_t bufferSize) override {
        if (closed) {
            return 0;
        }
        if (!seekable) {
            return bufferSize;
        }
        size_t n = std::min(bufferSize, totalSamples);
        remain -= n;
        return n;
    }

    void close() override {
        closed = true;
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
        return 1;
    }

    uint getSampleRate() const override {
        return sampleRate;
    }

    uint getBitsPerSample() const override {
        return 8;
    }

    bool isSeekable() const override {
        return seekable;
    }

    void seek(size_t position) override {
        if (closed || !seekable) {
            return;
        }
        position %= totalSamples;
        remain = totalSamples - position;
    }
};

void audio::initialize(bool enabled, AudioSettings& settings) {
    enabled = enabled && settings.enabled.get();
    if (enabled) {
        logger.info() << "initializing ALAudio backend";
        backend = ALAudio::create().release();
    }
    if (backend == nullptr) {
        if (enabled) {
            std::cerr << "could not to initialize audio" << std::endl;
        }
        logger.info() << "initializing NoAudio backend";
        backend = NoAudio::create().release();
    }
    struct {
        std::string name;
        NumberSetting* setting;
    } builtin_channels[] {
        {"master", &settings.volumeMaster},
        {"regular", &settings.volumeRegular},
        {"music", &settings.volumeMusic},
        {"ambient", &settings.volumeAmbient},
        {"ui", &settings.volumeUI}
    };
    for (auto& channel : builtin_channels) {
        create_channel(channel.name);
        objects_keeper.keepAlive(channel.setting->observe([=](auto value) {
            audio::get_channel(channel.name)->setVolume(value * value);
        }, true));
    }
}

std::unique_ptr<PCM> audio::load_PCM(const io::path& file, bool headerOnly) {
    if (!io::exists(file)) {
        throw std::runtime_error("file not found '" + file.string() + "'");
    }
    std::string ext = file.extension();
    if (ext == ".wav" || ext == ".WAV") {
        return wav::load_pcm(file, headerOnly);
    } else if (ext == ".ogg" || ext == ".OGG") {
        return ogg::load_pcm(file, headerOnly);
    }
    throw std::runtime_error("unsupported audio format");
}

std::unique_ptr<Sound> audio::load_sound(const io::path& file, bool keepPCM) {
    std::shared_ptr<PCM> pcm(
        load_PCM(file, !keepPCM && backend->isDummy()).release()
    );
    return create_sound(pcm, keepPCM);
}

std::unique_ptr<Sound> audio::create_sound(
    std::shared_ptr<PCM> pcm, bool keepPCM
) {
    return backend->createSound(std::move(pcm), keepPCM);
}

std::unique_ptr<PCMStream> audio::open_PCM_stream(const io::path& file) {
    std::string ext = file.extension();
    if (ext == ".wav" || ext == ".WAV") {
        return wav::create_stream(file);
    } else if (ext == ".ogg" || ext == ".OGG") {
        return ogg::create_stream(file);
    }
    throw std::runtime_error("unsupported audio stream format");
}

std::unique_ptr<Stream> audio::open_stream(
    const io::path& file, bool keepSource
) {
    if (!keepSource && backend->isDummy()) {
        auto header = load_PCM(file, true);
        // using void source sized as audio instead of actual audio file
        return open_stream(
            std::make_shared<PCMVoidSource>(
                header->totalSamples, header->sampleRate, header->seekable
            ),
            keepSource
        );
    }
    return open_stream(
        std::shared_ptr<PCMStream>(open_PCM_stream(file)), keepSource
    );
}

std::unique_ptr<Stream> audio::open_stream(
    std::shared_ptr<PCMStream> stream, bool keepSource
) {
    return backend->openStream(std::move(stream), keepSource);
}

void audio::set_listener(
    glm::vec3 position, glm::vec3 velocity, glm::vec3 lookAt, glm::vec3 up
) {
    backend->setListener(position, velocity, lookAt, up);
}

void remove_lower_priority_speaker(int priority) {
    for (auto it = speakers.begin(); it != speakers.end();) {
        if (it->second->getPriority() < priority && it->second->isPaused()) {
            it->second->stop();
            it = speakers.erase(it);
            return;
        }
        it++;
    }
    for (auto it = speakers.begin(); it != speakers.end();) {
        if (it->second->getPriority() < priority) {
            it->second->stop();
            it = speakers.erase(it);
            return;
        }
        it++;
    }
}

speakerid_t audio::play(
    Sound* sound,
    glm::vec3 position,
    bool relative,
    float volume,
    float pitch,
    bool loop,
    int priority,
    int channel
) {
    if (sound == nullptr) {
        return 0;
    }
    if (!sound->variants.empty()) {
        size_t index = rand() % (sound->variants.size() + 1);
        if (index < sound->variants.size()) {
            sound = sound->variants[index].get();
        }
    }
    auto speaker_ptr = sound->newInstance(priority, channel);
    if (speaker_ptr == nullptr) {
        remove_lower_priority_speaker(priority);
        speaker_ptr = sound->newInstance(priority, channel);
    }
    if (speaker_ptr == nullptr) {
        return 0;
    }
    auto speaker = speaker_ptr.get();
    speakerid_t id = nextId++;
    speakers.try_emplace(id, std::move(speaker_ptr));
    speaker->setPosition(position);
    speaker->setVolume(volume);
    speaker->setPitch(pitch);
    speaker->setLoop(loop);
    speaker->setRelative(relative);
    speaker->play();
    return id;
}

speakerid_t audio::play(
    const std::shared_ptr<Stream>& stream,
    glm::vec3 position,
    bool relative,
    float volume,
    float pitch,
    bool loop,
    int channel
) {
    auto speaker_ptr = stream->createSpeaker(loop, channel);
    if (speaker_ptr == nullptr) {
        remove_lower_priority_speaker(PRIORITY_HIGH);
        speaker_ptr = stream->createSpeaker(loop, channel);
    }
    if (speaker_ptr == nullptr) {
        return 0;
    }
    auto speaker = speaker_ptr.get();
    speakerid_t id = nextId++;
    streams.try_emplace(id, stream);
    speakers.try_emplace(id, std::move(speaker_ptr));
    stream->bindSpeaker(id);

    speaker->setPosition(position);
    speaker->setVolume(volume);
    speaker->setPitch(pitch);
    speaker->setLoop(false);
    speaker->setRelative(relative);
    speaker->play();
    return id;
}

speakerid_t audio::play_stream(
    const io::path& file,
    glm::vec3 position,
    bool relative,
    float volume,
    float pitch,
    bool loop,
    int channel
) {
    std::shared_ptr<Stream> stream(open_stream(file, false));
    return play(stream, position, relative, volume, pitch, loop, channel);
}

Speaker* audio::get_speaker(speakerid_t id) {
    auto found = speakers.find(id);
    if (found == speakers.end()) {
        return nullptr;
    }
    return found->second.get();
}

int audio::create_channel(const std::string& name) {
    int index = get_channel_index(name);
    if (index != -1) {
        return index;
    }
    channels.emplace_back(std::make_unique<Channel>(name));
    return channels.size() - 1;
}

int audio::get_channel_index(const std::string& name) {
    int index = 0;
    for (auto& channel : channels) {
        if (channel->getName() == name) {
            return index;
        }
        index++;
    }
    return -1;
}

Channel* audio::get_channel(int index) {
    if (index < 0 || index >= int(channels.size())) {
        return nullptr;
    }
    return channels.at(index).get();
}

Channel* audio::get_channel(const std::string& name) {
    return get_channel(get_channel_index(name));
}

std::shared_ptr<Stream> audio::get_associated_stream(speakerid_t id) {
    auto found = streams.find(id);
    if (found != streams.end()) {
        return found->second;
    }
    return nullptr;
}

size_t audio::count_speakers() {
    return speakers.size();
}

size_t audio::count_streams() {
    return streams.size();
}

void audio::update(double delta) {
    backend->update(delta);

    for (auto& entry : streams) {
        entry.second->update(delta);
    }

    for (auto it = speakers.begin(); it != speakers.end();) {
        auto speaker = it->second.get();
        int speakerChannel = speaker->getChannel();
        auto channel = get_channel(speakerChannel);
        if (channel != nullptr) {
            speaker->update(channel);
        }
        if (speaker->isStopped()) {
            streams.erase(it->first);
            it = speakers.erase(it);
        } else {
            it++;
        }
    }
}

void audio::reset_channel(int index) {
    auto channel = get_channel(index);
    if (channel == nullptr) {
        return;
    }
    for (auto& entry : speakers) {
        if (entry.second->getChannel() == index) {
            entry.second->stop();
        }
    }
    for (auto& entry : streams) {
        entry.second->update(0.0f);
    }
    for (auto it = speakers.begin(); it != speakers.end();) {
        auto speaker = it->second.get();
        int speakerChannel = speaker->getChannel();
        if (speakerChannel == index) {
            streams.erase(it->first);
            it = speakers.erase(it);
        } else {
            it++;
        }
    }
    if (channel->isPaused()) {
        channel->resume();
    }
}

void audio::close() {
    speakers.clear();
    delete backend;
    backend = nullptr;
    objects_keeper.clearKeepedObjects();
}
