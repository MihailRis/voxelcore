#include "Clock.hpp"

#include <cmath>

using namespace util;

Clock::Clock(int tickRate, int tickParts)
    : tickRate(tickRate), tickParts(tickParts) {
}

bool Clock::update(float delta) {
    tickTimer += delta;
    float delay = 1.0f / static_cast<float>(tickRate);
    if (tickTimer > delay || tickPartsUndone) {
        if (tickPartsUndone) {
            tickPartsUndone--;
        } else {
            tickTimer = std::fmod(tickTimer, delay);
            tickPartsUndone = tickParts - 1;
        }
        return true;
    }
    return false;
}

int Clock::getParts() const {
    return tickParts;
}

int Clock::getPart() const {
    return tickParts - tickPartsUndone - 1;
}

int Clock::getTickRate() const {
    return tickRate;
}

int Clock::getTickId() const {
    return tickId;
}
