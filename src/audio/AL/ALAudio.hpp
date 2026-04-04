#pragma once

#include <glm/glm.hpp>
#include <queue>
#include <string>
#include <array>
#include <unordered_map>
#include <vector>

#include "typedefs.hpp"
#include "audio/audio.hpp"
#include "audio/effects.hpp"

#include <AL/al.h>
#include <AL/alc.h>
#include <AL/alext.h>

struct AudioSettings;

namespace audio {
    struct ALBuffer;
    class ALAudio;
    class PCMStream;

    class ALSound : public Sound {
        ALAudio* al;
        uint buffer;
        std::shared_ptr<PCM> pcm;
        duration_t duration;
    public:
        ALSound(
            ALAudio* al,
            uint buffer,
            const std::shared_ptr<PCM>& pcm,
            bool keepPCM
        );
        ~ALSound();

        duration_t getDuration() const override {
            return duration;
        }

        std::shared_ptr<PCM> getPCM() const override {
            return pcm;
        }

        std::unique_ptr<Speaker> newInstance(int priority, int channel)
            const override;
    };

    class ALStream : public Stream {
        static inline constexpr size_t BUFFER_SIZE = 44100;

        ALAudio* al;
        std::shared_ptr<PCMStream> source;
        std::queue<uint> unusedBuffers;
        speakerid_t speaker = 0;
        bool keepSource;
        char buffer[BUFFER_SIZE];
        bool loop = false;
        bool stopOnEnd = false;

        bool preloadBuffer(uint buffer, bool loop);
        void unqueueBuffers(uint alsource);
        uint enqueueBuffers(uint alsource);
    public:
        size_t totalPlayedSamples = 0;

        ALStream(
            ALAudio* al, std::shared_ptr<PCMStream> source, bool keepSource
        );
        ~ALStream();

        std::shared_ptr<PCMStream> getSource() const override;
        void bindSpeaker(speakerid_t speakerid) override;
        std::unique_ptr<Speaker> createSpeaker(bool loop, int channel) override;
        speakerid_t getSpeaker() const override;
        void update(double delta) override;

        duration_t getTime() const override;
        void setTime(duration_t time) override;

        static inline constexpr uint STREAM_BUFFERS = 3;

        bool isStopOnEnd() const override;
        
        void setStopOnEnd(bool stopOnEnd) override;
    };

    class ALInputDevice : public InputDevice {
    public:
        ALInputDevice(
            ALAudio* al,
            ALCdevice* device,
            uint channels,
            uint bitsPerSample,
            uint sampleRate
        );
        ~ALInputDevice() override;

        void startCapture() override;
        void stopCapture() override;

        uint getChannels() const override;
        uint getSampleRate() const override;
        uint getBitsPerSample() const override;
        const std::string& getDeviceSpecifier() const override;

        size_t read(char* buffer, size_t bufferSize) override;
    private:
        ALCdevice* device;
        uint channels;
        uint bitsPerSample;
        uint sampleRate;
        std::string deviceSpecifier;
    };

    /// @brief AL source adapter
    class ALSpeaker : public Speaker {
        ALAudio* al;
        int priority;
        int channel;
        float volume = 0.0f;
    public:
        ALStream* stream = nullptr;
        bool manuallyStopped = true;
        bool paused = false;
        uint source;
        duration_t duration = 0.0f;

        ALSpeaker(ALAudio* al, uint source, int priority, int channel);
        ~ALSpeaker();

        void update(const Channel* channel) override;
        int getChannel() const override;

        State getState() const override;

        float getVolume() const override;
        void setVolume(float volume) override;

        float getPitch() const override;
        void setPitch(float pitch) override;

        bool isLoop() const override;
        void setLoop(bool loop) override;

        void play() override;
        void pause() override;
        void stop() override;

        duration_t getTime() const override;
        duration_t getDuration() const override;
        void setTime(duration_t time) override;

        void setPosition(glm::vec3 pos) override;
        glm::vec3 getPosition() const override;

        void setVelocity(glm::vec3 vel) override;
        glm::vec3 getVelocity() const override;

        void setRelative(bool relative) override;
        bool isRelative() const override;

        int getPriority() const override;

        bool isManuallyStopped() const override;
    };

    class ALAudio : public Backend {
        ALCdevice* device;
        ALCcontext* context;

        std::vector<uint> allsources;
        std::vector<uint> freesources;

        std::vector<uint> allbuffers;
        std::vector<uint> freebuffers;
        uint maxSources = 256;
        uint maxEffectSlots = 64;

        const AudioSettings& settings;

        bool initEffects();
    public:
        std::vector<uint> effectSlots;
        std::vector<uint> effects;
        std::array<uint, 1> filters;

        bool useEffects;

        ALAudio(
            ALCdevice* device,
            ALCcontext* context,
            bool effects,
            const AudioSettings& settings
        );
        ~ALAudio();

        uint getFreeSource();
        uint getFreeBuffer();
        void freeSource(uint source);
        void freeBuffer(uint buffer);

        std::unique_ptr<Sound> createSound(
            std::shared_ptr<PCM> pcm, bool keepPCM
        ) override;

        std::unique_ptr<Stream> openStream(
            std::shared_ptr<PCMStream> stream, bool keepSource
        ) override;

        std::unique_ptr<InputDevice> openInputDevice(
            const std::string& deviceName,
            uint sampleRate,
            uint channels,
            uint bitsPerSample
        ) override;

        std::vector<std::string> getOutputDeviceNames() override;
        std::vector<std::string> getInputDeviceNames() override;

        void setListener(
            glm::vec3 position,
            glm::vec3 velocity,
            glm::vec3 lookAt,
            glm::vec3 up
        ) override;

        void update(double delta) override;

        void setAcoustics(Acoustics acoustics) override;

        bool isDummy() const override {
            return false;
        }

        static std::unique_ptr<ALAudio> create(const AudioSettings& settings);
    };
}
