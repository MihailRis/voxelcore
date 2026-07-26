#include "Clock.hpp"

#include <cmath>
#include <algorithm>

using namespace util;

Clock::Clock(int tickRate, int tickParts)
    : tickRate(tickRate), tickParts(tickParts) {
}

int Clock::update(float delta) {
    tickTimer += delta;
    float delay = 1.0f / static_cast<float>(tickRate);
    if (tickTimer < delay / tickParts) {
        return 0;
    }
    int parts = tickTimer / (delay / tickParts);
    if (parts) {
        tickTimer -= parts * delay / tickParts;
        tickTimer = std::min<float>(tickTimer, delay);
    }
    currentTickPart += parts;
    if (currentTickPart >= tickParts) {
        currentTickPart %= tickParts;
    }
    return parts;
}

int Clock::getParts() const {
    return tickParts;
}

int Clock::getTickRate() const {
    return tickRate;
}

int Clock::getTickId() const {
    return tickId;
}

int Clock::convertPart(int index) const {
    return (tickParts - currentTickPart) % tickParts + index;
}
