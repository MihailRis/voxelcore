#pragma once

namespace util {
    class Clock {
        int tickRate;
        int tickParts;

        float tickTimer = 0.0f;
        int tickId = 0;
        int currentTickPart = 0;
    public:
        Clock(int tickRate, int tickParts);

        int update(float delta);

        int getParts() const;
        int getTickRate() const;
        int getTickId() const;
        int convertPart(int index) const;
    };
}
