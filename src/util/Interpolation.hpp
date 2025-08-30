#pragma once

#include <limits>
#include <glm/glm.hpp>

const float INTERPOLATION_DURATION = 0.15f; // seconds

namespace util {
    template<int N, typename T, bool angular=false>
    class VecInterpolation {
        bool enabled;
        glm::vec<N, T> prevPos {std::numeric_limits<T>::quiet_NaN()};
        glm::vec<N, T> nextPos {};
        mutable glm::vec<N, T> interpolationStep {};
        glm::vec<N, T> currentDuration {INTERPOLATION_DURATION};
        T timer = 0.0;
    public:
        VecInterpolation(bool enabled) : enabled(enabled) {}

        void refresh(const glm::vec<N, T>& position) {
            prevPos = getCurrent();
            nextPos = position;
            
            if constexpr (angular) {
                for (glm::length_t i = 0; i < N; i++) {
                    const float shortestAngle = std::fmod((std::fmod((nextPos[i] - prevPos[i]), 360.0f) + 540.0f), 360.0f) - 180.0f;
                    if (std::abs(interpolationStep[i]) > 90.0f) {
                        currentDuration[i] = INTERPOLATION_DURATION * (1.0f - std::abs(shortestAngle) / 180.0f);
                    } else {
                        currentDuration[i] = INTERPOLATION_DURATION;
                    }
                    interpolationStep[i] = shortestAngle / currentDuration[i];
                    if (glm::abs(nextPos[i]) > 180.0f) {
                        nextPos[i] += (nextPos[i] > 0.0f ? -360.0f : 360.0f);
                    }
                }
            } else {
                interpolationStep = (nextPos - prevPos) / currentDuration.x;
            }
            timer = 0.0;
        }

        void updateTimer(T delta) {
            timer += delta;
        }

        glm::vec<N, T> getCurrent() const {
            glm::vec<N, T> interpolated;
            for (glm::length_t i = 0; i < N; i++) {
                if (timer >= currentDuration[i] || std::isnan(prevPos[i])) {
                    interpolated[i] = nextPos[i];
                    interpolationStep[i] = 0.0f;
                } else {
                    interpolated[i] = prevPos[i] + interpolationStep[i] * timer;
                }
            }
            return interpolated;
        }

        bool isEnabled() const {
            return enabled;
        }

        void setEnabled(bool enabled) {
            this->enabled = enabled;
        }
    };
}
