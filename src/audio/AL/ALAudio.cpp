#include "ALAudio.hpp"

#include <string>
#include <utility>

#include "debug/Logger.hpp"
#include "alutil.hpp"
#include "../MemoryPCMStream.hpp"

static debug::Logger logger("al-audio");

using namespace audio;

const char* alc_error_to_string(ALCenum error) {
    switch (error) {
        case ALC_NO_ERROR:
            return "no error";
        case ALC_INVALID_DEVICE:
            return "invalid device handle";
        case ALC_INVALID_CONTEXT:
            return "invalid context handle";
        case ALC_INVALID_ENUM:
            return "invalid enum parameter passed to an ALC call";
        case ALC_INVALID_VALUE:
            return "invalid value parameter passed to an ALC call";
        case ALC_OUT_OF_MEMORY:
            return "out of memory";
        default:
            return "unknown ALC error";
    }
}

static bool check_alc_errors(ALCdevice* device, const char* context) {
    ALCenum error = alcGetError(device);
    if (error == ALC_NO_ERROR) {
        return false;
    }
    logger.error() << context << ": " << alc_error_to_string(error) << "("
                   << error << ")";
    return true;
}

ALSound::ALSound(
    ALAudio* al, uint buffer, const std::shared_ptr<PCM>& pcm, bool keepPCM
)
    : al(al), buffer(buffer) {
    duration = pcm->getDuration();
    if (keepPCM) {
        this->pcm = pcm;
    }
}

ALSound::~ALSound() {
    al->freeBuffer(buffer);
    buffer = 0;
}

std::unique_ptr<Speaker> ALSound::newInstance(int priority, int channel) const {
    uint source = al->getFreeSource();
    if (source == 0) {
        return nullptr;
    }
    AL_CHECK(alSourcei(source, AL_BUFFER, buffer));

    auto speaker = std::make_unique<ALSpeaker>(al, source, priority, channel);
    speaker->duration = duration;
    return speaker;
}

ALInputDevice::ALInputDevice(
    ALAudio* al,
    ALCdevice* device,
    uint channels,
    uint bitsPerSample,
    uint sampleRate
)
    : device(device),
      channels(channels),
      bitsPerSample(bitsPerSample),
      sampleRate(sampleRate) {
    const ALCchar* deviceName = alcGetString(device, ALC_CAPTURE_DEVICE_SPECIFIER);

    if (deviceName) {
        deviceSpecifier = std::string(deviceName);
    } else {
        logger.warning() << "could not retrieve input device specifier";
    }
}

ALInputDevice::~ALInputDevice() {
    alcCaptureCloseDevice(device);
    check_alc_errors(device, "alcCaptureCloseDevice");
}

void ALInputDevice::startCapture() {
    alcCaptureStart(device);
    check_alc_errors(device, "alcCaptureStart");
}

void ALInputDevice::stopCapture() {
    alcCaptureStop(device);
    check_alc_errors(device, "alcCaptureStop");
}

uint ALInputDevice::getChannels() const {
    return channels;
}

uint ALInputDevice::getSampleRate() const {
    return sampleRate;
}

uint ALInputDevice::getBitsPerSample() const {
    return bitsPerSample;
}

const std::string& ALInputDevice::getDeviceSpecifier() const {
    return deviceSpecifier;
}

size_t ALInputDevice::read(char* buffer, size_t bufferSize) {
    ALCint samplesCount = 0;
    alcGetIntegerv(device, ALC_CAPTURE_SAMPLES, sizeof(samplesCount), &samplesCount);
    check_alc_errors(device, "alcGetIntegerv(ALC_CAPTURE_SAMPLES)");
    size_t samplesRead = std::min<ALCsizei>(
        samplesCount, bufferSize / channels / (bitsPerSample >> 3)
    );
    alcCaptureSamples(device, buffer, samplesRead);
    check_alc_errors(device, "alcCaptureSamples");
    return samplesRead * channels * (bitsPerSample >> 3);
}

ALStream::ALStream(
    ALAudio* al, std::shared_ptr<PCMStream> source, bool keepSource
)
    : al(al), source(std::move(source)), keepSource(keepSource) {
}

ALStream::~ALStream() {
    bindSpeaker(0);
    source = nullptr;

    while (!unusedBuffers.empty()) {
        al->freeBuffer(unusedBuffers.front());
        unusedBuffers.pop();
    }
}

std::shared_ptr<PCMStream> ALStream::getSource() const {
    if (keepSource) {
        return source;
    } else {
        return nullptr;
    }
}

bool ALStream::preloadBuffer(uint buffer, bool loop) {
    size_t read = source->readFully(this->buffer, BUFFER_SIZE, loop);
    if (!read) return false;
    ALenum format =
        AL::to_al_format(source->getChannels(), source->getBitsPerSample());
    AL_CHECK(alBufferData(
        buffer, format, this->buffer, read, source->getSampleRate()
    ));
    return true;
}

std::unique_ptr<Speaker> ALStream::createSpeaker(bool loop, int channel) {
    this->loop = loop;
    uint free_source = al->getFreeSource();
    if (free_source == 0) {
        return nullptr;
    }
    for (uint i = 0; i < ALStream::STREAM_BUFFERS; i++) {
        uint free_buffer = al->getFreeBuffer();
        if (!preloadBuffer(free_buffer, loop)) {
            unusedBuffers.push(free_buffer);
        } else {
            AL_CHECK(alSourceQueueBuffers(free_source, 1, &free_buffer));
        }
    }
    return std::make_unique<ALSpeaker>(al, free_source, PRIORITY_HIGH, channel);
}

void ALStream::bindSpeaker(speakerid_t speakerid) {
    auto sp = audio::get_speaker(this->speaker);
    if (sp) {
        sp->stop();
    }
    this->speaker = speakerid;
    sp = audio::get_speaker(speakerid);
    if (sp) {
        auto alspeaker = dynamic_cast<ALSpeaker*>(sp);
        assert(alspeaker != nullptr); // backends must not be mixed
        alspeaker->stream = this;
        alspeaker->duration = source->getTotalDuration();
    }
}

speakerid_t ALStream::getSpeaker() const {
    return speaker;
}

void ALStream::unqueueBuffers(uint alsource) {
    uint processed = AL::getSourcei(alsource, AL_BUFFERS_PROCESSED);

    while (processed--) {
        uint bufferqueue;
        AL_CHECK(alSourceUnqueueBuffers(alsource, 1, &bufferqueue));
        unusedBuffers.push(bufferqueue);

        uint bps = source->getBitsPerSample()/ 8;
        uint channels = source->getChannels();

        ALint bufferSize;
        alGetBufferi(bufferqueue, AL_SIZE, &bufferSize);
        totalPlayedSamples += bufferSize / bps / channels;
        if (source->isSeekable()) {
            totalPlayedSamples %= source->getTotalSamples();
        }
    }
}

uint ALStream::enqueueBuffers(uint alsource) {
    uint preloaded = 0;
    if (!unusedBuffers.empty()) {
        uint firstBuffer = unusedBuffers.front();
        if (preloadBuffer(firstBuffer, loop)) {
            preloaded++;
            unusedBuffers.pop();
            AL_CHECK(alSourceQueueBuffers(alsource, 1, &firstBuffer));
        }
    }
    return preloaded;
}

void ALStream::update(double delta) {
    if (this->speaker == 0) {
        return;
    }
    auto speaker = audio::get_speaker(this->speaker);
    if (speaker == nullptr) {
        this->speaker = 0;
        return;
    }
    ALSpeaker* alspeaker = dynamic_cast<ALSpeaker*>(speaker);
    assert(alspeaker != nullptr);
    if (alspeaker->manuallyStopped) {
        this->speaker = 0;
        return;
    }

    uint alsource = alspeaker->source;

    unqueueBuffers(alsource);
    uint preloaded = enqueueBuffers(alsource);

    // alspeaker->stopped is assigned to false at ALSpeaker::play(...)
    if (speaker->isStopped() && !alspeaker->manuallyStopped) { //TODO: -V560 false-positive?
        if (preloaded) {
            speaker->play();
        } else if (isStopOnEnd()){
            speaker->stop();
        }
    }
}

duration_t ALStream::getTime() const {
    uint total = totalPlayedSamples;
    auto alspeaker =
        dynamic_cast<ALSpeaker*>(audio::get_speaker(this->speaker));
    if (alspeaker) {
        uint alsource = alspeaker->source;
        total +=
            static_cast<duration_t>(AL::getSourcef(alsource, AL_SAMPLE_OFFSET));
        if (source->isSeekable()) {
            total %= source->getTotalSamples();
        }
    }
    return total / static_cast<duration_t>(source->getSampleRate());
}

void ALStream::setTime(duration_t time) {
    if (!source->isSeekable()) return;
    uint sample = time * source->getSampleRate();
    source->seek(sample);
    auto alspeaker =
        dynamic_cast<ALSpeaker*>(audio::get_speaker(this->speaker));
    if (alspeaker) {
        bool paused = alspeaker->isPaused();
        AL_CHECK(alSourceStop(alspeaker->source));
        unqueueBuffers(alspeaker->source);
        totalPlayedSamples = sample;
        enqueueBuffers(alspeaker->source);
        AL_CHECK(alSourcePlay(alspeaker->source));
        if (paused) {
            AL_CHECK(alSourcePause(alspeaker->source));
        }
    } else {
        totalPlayedSamples = sample;
    }
}

bool ALStream::isStopOnEnd() const {
    return stopOnEnd;
}

void ALStream::setStopOnEnd(bool flag) {
    stopOnEnd = flag;
}

ALSpeaker::ALSpeaker(ALAudio* al, uint source, int priority, int channel)
    : al(al), priority(priority), channel(channel), source(source) {
}

ALSpeaker::~ALSpeaker() {
    if (source) {
        stop();
    }
}

void ALSpeaker::update(const Channel* channel) {
    if (source == 0) return;
    float gain = this->volume * channel->getVolume();
    AL_CHECK(alSourcef(source, AL_GAIN, gain));

    if (!paused) {
        if (isPaused() && !channel->isPaused()) {
            play();
        } else if (isPlaying() && channel->isPaused()) {
            AL_CHECK(alSourcePause(source));
        }
    }
}

int ALSpeaker::getChannel() const {
    return channel;
}

State ALSpeaker::getState() const {
    int state = AL::getSourcei(source, AL_SOURCE_STATE, AL_STOPPED);
    switch (state) {
        case AL_PLAYING:
            return State::playing;
        case AL_PAUSED:
            return State::paused;
        default:
            return State::stopped;
    }
}

float ALSpeaker::getVolume() const {
    return volume;
}

void ALSpeaker::setVolume(float volume) {
    this->volume = volume;
}

float ALSpeaker::getPitch() const {
    return AL::getSourcef(source, AL_PITCH);
}

void ALSpeaker::setPitch(float pitch) {
    AL_CHECK(alSourcef(source, AL_PITCH, pitch));
}

bool ALSpeaker::isLoop() const {
    return AL::getSourcei(source, AL_LOOPING) == AL_TRUE; //-V550
}

void ALSpeaker::setLoop(bool loop) {
    AL_CHECK(alSourcei(source, AL_LOOPING, loop ? AL_TRUE : AL_FALSE));
}

void ALSpeaker::play() {
    paused = false;
    manuallyStopped = false;
    auto p_channel = get_channel(this->channel);
    AL_CHECK(alSourcef(
        source,
        AL_GAIN,
        volume * p_channel->getVolume()
    ));
    AL_CHECK(alSourcePlay(source));
}

void ALSpeaker::pause() {
    paused = true;
    AL_CHECK(alSourcePause(source));
}

void ALSpeaker::stop() {
    manuallyStopped = true;
    if (source) {
        AL_CHECK(alSourceStop(source));

        uint processed = AL::getSourcei(source, AL_BUFFERS_PROCESSED);
        while (processed--) {
            uint buffer;
            AL_CHECK(alSourceUnqueueBuffers(source, 1, &buffer));
            al->freeBuffer(buffer);
        }
        AL_CHECK(alSourcei(source, AL_BUFFER, 0));
        al->freeSource(source);
        source = 0;
    }
}

duration_t ALSpeaker::getTime() const {
    if (stream) {
        return stream->getTime();
    }
    return static_cast<duration_t>(AL::getSourcef(source, AL_SEC_OFFSET));
}

duration_t ALSpeaker::getDuration() const {
    return duration;
}

void ALSpeaker::setTime(duration_t time) {
    if (stream) {
        return stream->setTime(time);
    }
    AL_CHECK(alSourcef(source, AL_SEC_OFFSET, static_cast<float>(time)));
}

void ALSpeaker::setPosition(glm::vec3 pos) {
    AL_CHECK(alSource3f(source, AL_POSITION, pos.x, pos.y, pos.z));
}

glm::vec3 ALSpeaker::getPosition() const {
    return AL::getSource3f(source, AL_POSITION);
}

void ALSpeaker::setVelocity(glm::vec3 vel) {
    AL_CHECK(alSource3f(source, AL_VELOCITY, vel.x, vel.y, vel.z));
}

glm::vec3 ALSpeaker::getVelocity() const {
    return AL::getSource3f(source, AL_VELOCITY);
}

void ALSpeaker::setRelative(bool relative) {
    AL_CHECK(
        alSourcei(source, AL_SOURCE_RELATIVE, relative ? AL_TRUE : AL_FALSE)
    );
}

bool ALSpeaker::isRelative() const {
    return AL::getSourcei(source, AL_SOURCE_RELATIVE) == AL_TRUE; //-V550
}

int ALSpeaker::getPriority() const {
    return priority;
}


bool ALSpeaker::isManuallyStopped() const {
    return manuallyStopped;
}

static bool alc_enumeration_ext = false;

ALAudio::ALAudio(ALCdevice* device, ALCcontext* context)
    : device(device), context(context) {
    ALCint size;
    alcGetIntegerv(device, ALC_ATTRIBUTES_SIZE, 1, &size);
    std::vector<ALCint> attrs(size);
    alcGetIntegerv(device, ALC_ALL_ATTRIBUTES, size, &attrs[0]);
    for (size_t i = 0; i < attrs.size(); ++i) {
        if (attrs[i] == ALC_MONO_SOURCES) {
            logger.info() << "max mono sources: " << attrs[i + 1];
            maxSources = attrs[i + 1];
        }
    }
    auto outputDevices = getOutputDeviceNames();
    logger.info() << "output devices:";
    for (auto& name : outputDevices) {
        logger.info() << "  " << name;
    }

    auto inputDevices = getInputDeviceNames();
    logger.info() << "input devices:";
    for (auto& name : inputDevices) {
        logger.info() << "  " << name;
    }
}

ALAudio::~ALAudio() {
    for (uint source : allsources) {
        int state = AL::getSourcei(source, AL_SOURCE_STATE);
        if (state == AL_PLAYING || state == AL_PAUSED) {
            AL_CHECK(alSourceStop(source));
        }
        AL_CHECK(alDeleteSources(1, &source));
    }

    for (uint buffer : allbuffers) {
        AL_CHECK(alDeleteBuffers(1, &buffer));
    }

    alcMakeContextCurrent(nullptr);
    check_alc_errors(device, "alcMakeContextCurrent");
    alcDestroyContext(context);
    check_alc_errors(device, "alcDestroyContext");
    if (!alcCloseDevice(device)) {
        logger.error() << "device not closed!";
    }
    device = nullptr;
    context = nullptr;
}

std::unique_ptr<Sound> ALAudio::createSound(
    std::shared_ptr<PCM> pcm, bool keepPCM
) {
    auto format = AL::to_al_format(pcm->channels, pcm->bitsPerSample);
    uint buffer = getFreeBuffer();
    AL_CHECK(alBufferData(
        buffer, format, pcm->data.data(), pcm->data.size(), pcm->sampleRate
    ));
    return std::make_unique<ALSound>(this, buffer, pcm, keepPCM);
}

std::unique_ptr<Stream> ALAudio::openStream(
    std::shared_ptr<PCMStream> stream, bool keepSource
) {
    return std::make_unique<ALStream>(this, stream, keepSource);
}

std::vector<std::string> ALAudio::getInputDeviceNames() {
    std::vector<std::string> devices;
    
    if (!alc_enumeration_ext) {
        logger.warning() << "enumeration extension is not available";
        return devices;
    }

    auto deviceList = alcGetString(nullptr, ALC_CAPTURE_DEVICE_SPECIFIER);
    if (deviceList == nullptr) {
        logger.warning() << "no input devices found";
        return devices;
    }
    while (*deviceList) {
        std::string deviceName(deviceList);
        devices.push_back(deviceName);
        deviceList += deviceName.length() + 1;
    }
    
    return devices;
}

std::vector<std::string> ALAudio::getOutputDeviceNames() {
    std::vector<std::string> devices;
    
    if (!alc_enumeration_ext) {
        logger.warning() << "enumeration extension is not available";
        return devices;
    }

    auto deviceList = alcGetString(nullptr, ALC_ALL_DEVICES_SPECIFIER);
    if (deviceList == nullptr) {
        logger.warning() << "no input devices found";
        return devices;
    }
    while (*deviceList) {
        std::string deviceName(deviceList);
        devices.push_back(deviceName);
        deviceList += deviceName.length() + 1;
    }
    
    return devices;
}

std::unique_ptr<InputDevice> ALAudio::openInputDevice(
    const std::string& deviceName, uint sampleRate, uint channels, uint bitsPerSample
) {
    uint bps = bitsPerSample >> 3;
    ALCdevice* device = alcCaptureOpenDevice(
        deviceName.empty() ? nullptr : deviceName.c_str(),
        sampleRate,
        AL::to_al_format(channels, bitsPerSample),
        sampleRate * channels * bps / 8
    );
    if (check_alc_errors(device, "alcCaptureOpenDevice"))
        return nullptr;

    return std::make_unique<ALInputDevice>(
        this, device, channels, bitsPerSample, sampleRate
    );
}

std::unique_ptr<ALAudio> ALAudio::create() {
    alc_enumeration_ext = alcIsExtensionPresent(nullptr, "ALC_ENUMERATION_EXT");

    ALCdevice* device = alcOpenDevice(nullptr);
    if (device == nullptr) return nullptr;
    ALCcontext* context = alcCreateContext(device, nullptr);
    if (!alcMakeContextCurrent(context)) {
        alcCloseDevice(device);
        return nullptr;
    }
    AL_CHECK();
    logger.info() << "initialized";
    return std::make_unique<ALAudio>(device, context);
}

uint ALAudio::getFreeSource() {
    if (!freesources.empty()) {
        uint source = freesources.back();
        freesources.pop_back();
        return source;
    }
    if (allsources.size() == maxSources) {
        logger.error() << "attempted to create new source, but limit is "
                       << maxSources;
        return 0;
    }
    ALuint id;
    alGenSources(1, &id);
    if (!AL_GET_ERROR()) {
        return 0;
    }
    allsources.push_back(id);
    return id;
}

uint ALAudio::getFreeBuffer() {
    if (!freebuffers.empty()) {
        uint buffer = freebuffers.back();
        freebuffers.pop_back();
        return buffer;
    }
    ALuint id;
    alGenBuffers(1, &id);
    if (!AL_GET_ERROR()) {
        return 0;
    }

    allbuffers.push_back(id);
    return id;
}

void ALAudio::freeSource(uint source) {
    freesources.push_back(source);
}

void ALAudio::freeBuffer(uint buffer) {
    freebuffers.push_back(buffer);
}

void ALAudio::setListener(
    glm::vec3 position, glm::vec3 velocity, glm::vec3 at, glm::vec3 up
) {
    ALfloat listenerOri[] = {at.x, at.y, at.z, up.x, up.y, up.z};

    AL_CHECK(alListener3f(AL_POSITION, position.x, position.y, position.z));
    AL_CHECK(alListener3f(AL_VELOCITY, velocity.x, velocity.y, velocity.z));
    AL_CHECK(alListenerfv(AL_ORIENTATION, listenerOri));
    AL_CHECK(alListenerf(AL_GAIN, get_channel(0)->getVolume()));
}

void ALAudio::update(double) {
}
