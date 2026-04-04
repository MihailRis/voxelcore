#pragma once

#include <glm/glm.hpp>
#include <memory>
#include <vector>

#include "typedefs.hpp"
#include "settings.hpp"
#include "io/fwd.hpp"
#include "effects.hpp"

namespace audio {
    /// @brief playing speaker uid
    using speakerid_t = int64_t;

    /// @brief duration unit is a second
    using duration_t = double;

    /// @brief not important sounds (steps, random noises)
    constexpr inline int PRIORITY_LOW = 0;

    /// @brief default sounds priority
    constexpr inline int PRIORITY_NORMAL = 5;

    /// @brief streams and important sounds
    constexpr inline int PRIORITY_HIGH = 10;

    constexpr inline size_t MAX_INPUT_SAMPLES = 22050;

    inline std::string_view DEVICE_NONE = "none";

    class Speaker;

    /// @brief Audio speaker states
    enum class State { playing, paused, stopped };

    /// @brief Mixer channel controls speakers volume and effects
    /// There is main channel 'master' and sub-channels like 'regular', 'music',
    /// 'ambient'...
    class Channel {
        /// @brief Channel name
        std::string name;
        /// @brief Channel volume setting
        float volume = 1.0f;
        bool paused = false;
        bool effects;
    public:
        Channel(std::string name, bool effects);

        /// @brief Get channel volume
        float getVolume() const;

        /// @brief Set channel volume
        /// @param volume value in range [0.0, 1.0]
        void setVolume(float volume);

        /// @brief Get channel name
        const std::string& getName() const;

        inline void setPaused(bool flag) {
            if (flag == paused) return;
            if (flag) {
                pause();
            } else {
                resume();
            }
        }

        /// @brief Pause all speakers in channel
        void pause();

        /// @brief Unpause all speakers paused by this channel
        void resume();

        /// @brief Check if the channel is paused
        bool isPaused() const;

        /// @brief Check if the channel uses common acoustic effects / filters
        bool isEffectsApplied() const;
    };

    /// @brief Pulse-code modulation data
    struct PCM {
        /// @brief May contain 8 bit and 16 bit PCM data
        std::vector<char> data;
        /// @brief Total number of mono samples (sampleRate per second)
        size_t totalSamples;
        /// @brief Track channels number (1 - mono, 2 - stereo)
        uint8_t channels;
        /// @brief Sample bit depth (8 and 16 supported)
        /// does not depend on the channels number
        uint8_t bitsPerSample;
        /// @brief Number of mono samples per second (example: 44100)
        uint sampleRate;
        /// @brief Audio source is seekable
        bool seekable;

        PCM(std::vector<char> data,
            size_t totalSamples,
            uint8_t channels,
            uint8_t bitsPerSample,
            uint sampleRate,
            bool seekable)
            : data(std::move(data)),
              totalSamples(totalSamples),
              channels(channels),
              bitsPerSample(bitsPerSample),
              sampleRate(sampleRate),
              seekable(seekable) {
        }

        /// @brief Get total audio duration
        /// @return duration in seconds
        inline duration_t getDuration() const {
            return static_cast<duration_t>(totalSamples) /
                   static_cast<duration_t>(sampleRate);
        }
    };

    class InputDevice {
    public:
        virtual ~InputDevice() {};

        virtual void startCapture() = 0;
        virtual void stopCapture() = 0;

        /// @brief Get number of audio channels
        /// @return 1 if mono, 2 if stereo
        virtual uint getChannels() const = 0;
        /// @brief Get audio sampling frequency
        /// @return number of mono samples per second
        virtual uint getSampleRate() const = 0;
        /// @brief Get number of bits per mono sample
        /// @return 8 or 16
        virtual uint getBitsPerSample() const = 0;

        /// @brief Read available data to buffer.
        /// @return size of data received or PCMStream::ERROR in case of error
        virtual size_t read(char* buffer, size_t bufferSize) = 0;

        /// @brief Get device specifier string
        virtual const std::string& getDeviceSpecifier() const = 0;
    };

    /// @brief audio::PCMStream is a data source for audio::Stream
    class PCMStream {
    public:
        virtual ~PCMStream() {};

        /// @brief Read samples data to buffer
        /// @param buffer destination buffer
        /// @param bufferSize destination buffer size
        /// @param loop loop stream (seek to start when end reached)
        /// @return size of data received
        /// (always equals bufferSize if seekable and looped)
        virtual size_t readFully(char* buffer, size_t bufferSize, bool loop);

        /// @brief Read available data to buffer
        /// @param buffer destination buffer
        /// @param bufferSize destination buffer size
        /// @return count of received bytes or PCMStream::ERROR
        virtual size_t read(char* buffer, size_t bufferSize) = 0;

        /// @brief Close stream
        virtual void close() = 0;

        /// @brief Check if stream is open
        virtual bool isOpen() const = 0;

        /// @brief Get total samples number if seekable or 0
        virtual size_t getTotalSamples() const = 0;

        /// @brief Get total audio track duration if seekable or 0.0
        virtual duration_t getTotalDuration() const = 0;

        /// @brief Get number of audio channels
        /// @return 1 if mono, 2 if stereo
        virtual uint getChannels() const = 0;

        /// @brief Get audio sampling frequency
        /// @return number of mono samples per second
        virtual uint getSampleRate() const = 0;

        /// @brief Get number of bits per mono sample
        /// @return 8 or 16
        virtual uint getBitsPerSample() const = 0;

        /// @brief Check if the stream does support seek feature
        virtual bool isSeekable() const = 0;

        /// @brief Move playhead to the selected sample number
        /// @param position selected sample number
        virtual void seek(size_t position) = 0;

        /// @brief Value returning by read(...) in case of error
        static const size_t ERROR = -1;
    };

    /// @brief Audio streaming interface
    class Stream {
    public:
        virtual ~Stream() {};

        /// @brief Get pcm data source
        /// @return PCM stream or nullptr if audio::openStream
        /// keepSource argument is set to false
        virtual std::shared_ptr<PCMStream> getSource() const = 0;

        /// @brief Create new speaker bound to the Stream
        /// and having high priority
        /// @param loop is stream looped (required for correct buffers preload)
        /// @param channel channel index
        /// @return speaker id or 0
        virtual std::unique_ptr<Speaker> createSpeaker(
            bool loop, int channel
        ) = 0;

        /// @brief Unbind previous speaker and bind new speaker to the stream
        /// @param speaker speaker id or 0 if all you need is unbind speaker
        virtual void bindSpeaker(speakerid_t speaker) = 0;

        /// @brief Get id of the bound speaker
        /// @return speaker id or 0 if no speaker bound
        virtual speakerid_t getSpeaker() const = 0;

        /// @brief Update stream state (preload samples if needed)
        /// @param delta time elapsed since the last update
        virtual void update(double delta) = 0;

        /// @brief Get current stream time
        virtual duration_t getTime() const = 0;

        /// @brief Set playhead to the selected time
        /// @param time selected time
        virtual void setTime(duration_t time) = 0;

        virtual bool isStopOnEnd() const = 0;
        virtual void setStopOnEnd(bool stopOnEnd) = 0;
    };

    /// @brief Sound is an audio asset that supposed to support many
    /// simultaneously playing instances (speakers).
    /// So it's audio data is stored in memory.
    class Sound {
    public:
        /// @brief Sound variants will be chosen randomly to play
        std::vector<std::shared_ptr<Sound>> variants;

        virtual ~Sound() {
        }

        /// @brief Get sound duration
        /// @return duration in seconds (>= 0.0)
        virtual duration_t getDuration() const = 0;

        /// @brief Get sound PCM data
        /// @return PCM data or nullptr
        virtual std::shared_ptr<PCM> getPCM() const = 0;

        /// @brief Create new sound instance
        /// @param priority instance priority. High priority instance can
        /// take out speaker from low priority instance
        /// @param channel channel index
        /// @return new speaker with sound bound or nullptr
        /// if all speakers are in use
        virtual std::unique_ptr<Speaker> newInstance(int priority, int channel)
            const = 0;
    };

    /// @brief Audio source controller interface.
    /// @attention Speaker is not supposed to be reused
    class Speaker {
    public:
        virtual ~Speaker() {
        }

        /// @brief Synchronize the speaker with channel settings
        /// @param channel speaker channel
        virtual void update(const Channel* channel) = 0;

        /// @brief Check speaker channel index
        virtual int getChannel() const = 0;

        /// @brief Get current speaker state
        /// @return speaker state
        virtual State getState() const = 0;

        /// @brief Get speaker audio gain
        /// @return speaker audio gain value
        virtual float getVolume() const = 0;

        /// @brief Set speaker audio gain (must be positive)
        /// @param volume new gain value
        virtual void setVolume(float volume) = 0;

        /// @brief Get speaker pitch multiplier
        /// @return pitch multiplier
        virtual float getPitch() const = 0;

        /// @brief Set speaker pitch multiplier
        /// @param pitch new pitch multiplier (must be positive)
        virtual void setPitch(float pitch) = 0;

        /// @brief Check if speaker audio is in loop
        /// @return true if audio is in loop
        virtual bool isLoop() const = 0;

        /// @brief Enable/disable audio loop
        /// @param loop loop mode
        virtual void setLoop(bool loop) = 0;

        /// @brief Play, replay or resume audio
        virtual void play() = 0;

        /// @brief Pause playing audio and keep speaker alive
        virtual void pause() = 0;

        /// @brief Stop and destroy speaker
        virtual void stop() = 0;

        /// @brief Get current time position of playing audio
        /// @return time position in seconds
        virtual duration_t getTime() const = 0;

        /// @brief Get playing audio total duration
        virtual duration_t getDuration() const = 0;

        /// @brief Set playing audio time position
        /// @param time time position in seconds
        virtual void setTime(duration_t time) = 0;

        /// @brief Set speaker 3D position in the world
        /// @param pos new position
        virtual void setPosition(glm::vec3 pos) = 0;

        /// @brief Get speaker 3D position in the world
        /// @return position
        virtual glm::vec3 getPosition() const = 0;

        /// @brief Set speaker movement velocity used for Doppler effect
        /// @param vel velocity vector
        virtual void setVelocity(glm::vec3 vel) = 0;

        /// @brief Get speaker movement velocity used for Doppler effect
        /// @return velocity vector
        virtual glm::vec3 getVelocity() const = 0;

        /// @brief Get speaker priority
        /// @return speaker priority value
        virtual int getPriority() const = 0;

        /// @brief Determines if the position is relative to the listener
        /// @param relative true - relative to the listener (default: false)
        virtual void setRelative(bool relative) = 0;

        /// @brief Determines if the position is relative to the listener
        virtual bool isRelative() const = 0;

        /// @brief Check if speaker is playing
        inline bool isPlaying() const {
            return getState() == State::playing;
        }

        /// @brief Check if speaker is paused
        inline bool isPaused() const {
            return getState() == State::paused;
        }

        /// @brief Check if speaker is stopped
        inline bool isStopped() const {
            return getState() == State::stopped;
        }

        virtual bool isManuallyStopped() const = 0;
    };

    class Backend {
    public:
        virtual ~Backend() {};

        virtual std::unique_ptr<Sound> createSound(
            std::shared_ptr<PCM> pcm, bool keepPCM
        ) = 0;
        virtual std::unique_ptr<Stream> openStream(
            std::shared_ptr<PCMStream> stream, bool keepSource
        ) = 0;
        virtual std::unique_ptr<InputDevice> openInputDevice(
            const std::string& deviceName,
            uint sampleRate,
            uint channels,
            uint bitsPerSample
        ) = 0;
        virtual void setListener(
            glm::vec3 position,
            glm::vec3 velocity,
            glm::vec3 lookAt,
            glm::vec3 up
        ) = 0;
        virtual std::vector<std::string> getInputDeviceNames() = 0;
        virtual std::vector<std::string> getOutputDeviceNames() = 0;
        virtual void update(double delta) = 0;
        virtual void setAcoustics(Acoustics acoustics) = 0;

        /// @brief Check if backend is an abstraction that does not internally
        /// work with actual audio data or play anything
        virtual bool isDummy() const = 0;
    };

    /// @brief Initialize audio system or use no audio mode
    /// @param enabled try to initialize actual audio
    void initialize(bool enabled, bool inputEnabled, AudioSettings& settings);

    /// @brief Load audio file info and PCM data
    /// @param file audio file
    /// @param headerOnly read header only
    /// @throws std::runtime_error if I/O error ocurred or format is unknown
    /// @return PCM audio data
    std::unique_ptr<PCM> load_PCM(const io::path& file, bool headerOnly);

    /// @brief Load sound from file
    /// @param file audio file path
    /// @param keepPCM store PCM data in sound to make it accessible with
    /// Sound::getPCM
    /// @throws std::runtime_error if I/O error ocurred or format is unknown
    /// @return new Sound instance
    std::unique_ptr<Sound> load_sound(const io::path& file, bool keepPCM);

    /// @brief Create new sound from PCM data
    /// @param pcm PCM data
    /// @param keepPCM store PCM data in sound to make it accessible with
    /// Sound::getPCM
    /// @return new Sound instance
    std::unique_ptr<Sound> create_sound(std::shared_ptr<PCM> pcm, bool keepPCM);

    /// @brief Open new PCM stream from file
    /// @param file audio file path
    /// @throws std::runtime_error if I/O error ocurred or format is unknown
    /// @return new PCMStream instance
    std::unique_ptr<PCMStream> open_PCM_stream(const io::path& file);

    /// @brief Open new audio stream from file
    /// @param file audio file path
    /// @param keepSource store PCMStream in stream to make it accessible with
    /// Stream::getSource
    /// @return new Stream instance
    std::unique_ptr<Stream> open_stream(const io::path& file, bool keepSource);

    /// @brief Open new audio stream from source
    /// @param stream PCM data source
    /// @param keepSource store PCMStream in stream to make it accessible with
    /// Stream::getSource
    /// @return new Stream instance
    std::unique_ptr<Stream> open_stream(
        std::shared_ptr<PCMStream> stream, bool keepSource
    );

    /// @brief Open audio input device
    /// @param sampleRate sample rate
    /// @param channels channels count (1 - mono, 2 - stereo)
    /// @param bitsPerSample number of bits per sample (8 or 16)
    /// @return new InputDevice instance or nullptr
    std::unique_ptr<InputDevice> open_input_device(
        const std::string& deviceName,
        uint sampleRate,
        uint channels,
        uint bitsPerSample
    );

    /// @brief Retrieve names of available audio input devices
    /// @return list of device names
    std::vector<std::string> get_input_devices_names();

    /// @brief Retrieve names of available audio output devices
    /// @return list of device names
    std::vector<std::string> get_output_devices_names();

    void set_input_device(const std::string& deviceName);

    /// @brief Configure 3D listener
    /// @param position listener position
    /// @param velocity listener velocity (used for Doppler effect)
    /// @param lookAt point the listener look at
    /// @param up camera up vector
    void set_listener(
        glm::vec3 position, glm::vec3 velocity, glm::vec3 lookAt, glm::vec3 up
    );

    /// @brief Play 3D sound in the world
    /// @param sound target sound
    /// @param position sound world position
    /// @param relative position speaker relative to listener
    /// @param volume sound volume [0.0-1.0]
    /// @param pitch sound pitch multiplier [0.0-...]
    /// @param loop loop sound
    /// @param priority sound priority
    /// (PRIORITY_LOW, PRIORITY_NORMAL, PRIORITY_HIGH)
    /// @param channel channel index
    /// @return speaker id or 0
    speakerid_t play(
        Sound* sound,
        glm::vec3 position,
        bool relative,
        float volume,
        float pitch,
        bool loop,
        int priority,
        int channel
    );

    /// @brief Play stream
    /// @param stream target stream
    /// @param position stream world position
    /// @param relative position speaker relative to listener
    /// @param volume stream volume [0.0-1.0]
    /// @param pitch stream pitch multiplier [0.0-...]
    /// @param loop loop stream
    /// @param channel channel index
    /// @return speaker id or 0
    speakerid_t play(
        const std::shared_ptr<Stream>& stream,
        glm::vec3 position,
        bool relative,
        float volume,
        float pitch,
        bool loop,
        int channel
    );

    /// @brief Play stream from file
    /// @param file audio file path
    /// @param position stream world position
    /// @param relative position speaker relative to listener
    /// @param volume stream volume [0.0-1.0]
    /// @param pitch stream pitch multiplier [0.0-...]
    /// @param loop loop stream
    /// @param channel channel index
    /// @return speaker id or 0
    speakerid_t play_stream(
        const io::path& file,
        glm::vec3 position,
        bool relative,
        float volume,
        float pitch,
        bool loop,
        int channel
    );

    /// @brief Get speaker by id
    /// @param id speaker id
    /// @return speaker or nullptr
    Speaker* get_speaker(speakerid_t id);

    /// @brief Create new channel.
    /// All non-builtin channels will be destroyed on audio::reset() call
    /// @param name channel name
    /// @param effects use common acoustic effects / filters
    /// @return new channel index
    int create_channel(const std::string& name, bool effects);

    /// @brief Get channel index by name
    /// @param name channel name
    /// @return channel index or -1
    int get_channel_index(const std::string& name);

    /// @brief Get channel by index. 0 - is master channel
    /// @param index channel index
    /// @return channel or nullptr
    Channel* get_channel(int index);

    /// @brief Get channel by name.
    /// @param name channel name
    /// @return channel or nullptr
    Channel* get_channel(const std::string& name);

    /// @brief Get stream associated with speaker
    /// @param id speaker id
    /// @return stream or nullptr
    std::shared_ptr<Stream> get_associated_stream(speakerid_t id);

    /// @brief Get alive speakers number (including paused)
    size_t count_speakers();

    /// @brief Get playing streams number (including paused)
    size_t count_streams();

    /// @brief Update audio streams and sound instanced
    /// @param delta time elapsed since the last update (seconds)
    void update(double delta);

    void set_acoustics(Acoustics acoustics);

    /// @brief Stop all playing audio in channel, reset channel state
    void reset_channel(int channel);

    InputDevice* get_input_device();

    /// @brief Finalize audio system
    void close();
};
