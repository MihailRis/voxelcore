#pragma once

#include "audio.hpp"

namespace audio {
    class NoSound : public Sound {
        std::shared_ptr<PCM> pcm;
        duration_t duration;
    public:
        NoSound(const std::shared_ptr<PCM>& pcm, bool keepPCM);
        ~NoSound() {
        }

        duration_t getDuration() const override {
            return duration;
        }

        std::shared_ptr<PCM> getPCM() const override {
            return pcm;
        }

        std::unique_ptr<Speaker> newInstance(int priority, int channel)
            const override {
            return nullptr;
        }
    };

    class NoStream : public Stream {
        std::shared_ptr<PCMStream> source;
        duration_t duration;
    public:
        NoStream(const std::shared_ptr<PCMStream>& source, bool keepSource) {
            duration = source->getTotalDuration();
            if (keepSource) {
                this->source = source;
            }
        }

        std::shared_ptr<PCMStream> getSource() const override {
            return source;
        }

        void bindSpeaker(speakerid_t speaker) override {
        }

        std::unique_ptr<Speaker> createSpeaker(bool loop, int channel)
            override {
            return nullptr;
        }

        speakerid_t getSpeaker() const override {
            return 0;
        }

        void update(double delta) override {
        }

        duration_t getTime() const override {
            return 0.0;
        }

        void setTime(duration_t time) override {
        }

        bool isStopOnEnd() const override {
            return false;
        }

        void setStopOnEnd(bool stopOnEnd) override {
        }
    };

    class NoAudio : public Backend {
    public:
        ~NoAudio() {
        }

        std::unique_ptr<Sound> createSound(
            std::shared_ptr<PCM> pcm, bool keepPCM
        ) override;

        std::unique_ptr<Stream> openStream(
            std::shared_ptr<PCMStream> stream, bool keepSource
        ) override;

        std::unique_ptr<InputDevice> openInputDevice(
            const std::string& deviceName, uint sampleRate, uint channels, uint bitsPerSample
        ) override {
            return nullptr;
        }

        std::vector<std::string> getInputDeviceNames() override {
            return {};
        }
        std::vector<std::string> getOutputDeviceNames() override {
            return {};
        }

        void setListener(
            glm::vec3 position, glm::vec3 velocity, glm::vec3 at, glm::vec3 up
        ) override {
        }

        void update(double delta) override {
        }

        bool isDummy() const override {
            return true;
        }

        static std::unique_ptr<NoAudio> create();
    };
}
