#pragma once

#include <vector>
#include <stdexcept>
#include <functional>
#include <glm/glm.hpp>

namespace util {

    template<class T, typename TCoord=int>
    class AreaMap2D {
    public:
        using OutCallback = std::function<void(TCoord, TCoord, const T&)>;
    private:
        TCoord offsetX = 0, offsetY = 0;
        TCoord sizeX, sizeY;
        std::vector<T> firstBuffer;
        std::vector<T> secondBuffer;
        OutCallback outCallback;

        size_t valuesCount = 0;
    
        void translate(TCoord dx, TCoord dy) {
            if (dx == 0 && dy == 0) {
                return;
            }
            std::fill(secondBuffer.begin(), secondBuffer.end(), T{});
            for (TCoord y = 0; y < sizeY; y++) {
                for (TCoord x = 0; x < sizeX; x++) {
                    auto value = std::move(firstBuffer[y * sizeX + x]);
                    auto nx = x - dx;
                    auto ny = y - dy;
                    if (value == T{}) {
                        continue;
                    }
                    if (nx < 0 || ny < 0 || nx >= sizeX || ny >= sizeY) {
                        if (outCallback) {
                            outCallback(x + offsetX, y + offsetY, value);
                        }
                        valuesCount--;
                        continue;
                    }
                    secondBuffer[ny * sizeX + nx] = std::move(value);
                }
            }
            std::swap(firstBuffer, secondBuffer);
            offsetX += dx;
            offsetY += dy;
        }
    public:
        AreaMap2D(TCoord width, TCoord height)
            : sizeX(width), sizeY(height), 
              firstBuffer(width * height), secondBuffer(width * height) {
        }

        const T* getIf(TCoord x, TCoord y) const {
            auto lx = x - offsetX;
            auto ly = y - offsetY;
            if (lx < 0 || ly < 0 || lx >= sizeX || ly >= sizeY) {
                return nullptr;
            }
            return &firstBuffer[ly * sizeX + lx];
        }

        T get(TCoord x, TCoord y) const {
            auto lx = x - offsetX;
            auto ly = y - offsetY;
            if (lx < 0 || ly < 0 || lx >= sizeX || ly >= sizeY) {
                return T{};
            }
            return firstBuffer[ly * sizeX + lx];
        }

        T get(TCoord x, TCoord y, const T& def) const {
            if (auto ptr = getIf(x, y)) {
                const auto& value = *ptr;
                if (value == T{}) {
                    return def;
                }
                return value;
            }
            return def;
        }

        bool isInside(TCoord x, TCoord y) const {
            auto lx = x - offsetX;
            auto ly = y - offsetY;
            return !(lx < 0 || ly < 0 || lx >= sizeX || ly >= sizeY);
        }

        const T& require(TCoord x, TCoord y) const {
            auto lx = x - offsetX;
            auto ly = y - offsetY;
            if (lx < 0 || ly < 0 || lx >= sizeX || ly >= sizeY) {
                throw std::invalid_argument("position is out of window");
            }
            return firstBuffer[ly * sizeX + lx];
        }

        bool set(TCoord x, TCoord y, T value) {
            auto lx = x - offsetX;
            auto ly = y - offsetY;
            if (lx < 0 || ly < 0 || lx >= sizeX || ly >= sizeY) {
                return false;
            }
            auto& element = firstBuffer[ly * sizeX + lx];
            if (value && !element) {
                valuesCount++;
            }
            if (element && !value) {
                valuesCount--;
            }
            element = std::move(value);
            return true;
        }

        void remove(TCoord x, TCoord y) {
            auto lx = x - offsetX;
            auto ly = y - offsetY;
            if (lx < 0 || ly < 0 || lx >= sizeX || ly >= sizeY) {
                return;
            }
            if (outCallback)
                outCallback(x, y, firstBuffer[ly * sizeX + lx]);
            firstBuffer[ly * sizeX + lx] = T{};
        }

        void setOutCallback(const OutCallback& callback) {
            outCallback = callback;
        }

        void resize(TCoord newSizeX, TCoord newSizeY) {
            if (newSizeX < sizeX) {
                TCoord delta = sizeX - newSizeX;
                translate(delta / 2, 0);
                translate(-delta, 0);
                translate(delta, 0);
            }
            if (newSizeY < sizeY) {
                TCoord delta = sizeY - newSizeY;
                translate(0, delta / 2);
                translate(0, -delta);
                translate(0, delta);
            }
            const TCoord newVolume = newSizeX * newSizeY;
            std::vector<T> newFirstBuffer(newVolume);
            std::vector<T> newSecondBuffer(newVolume);
            for (TCoord y = 0; y < sizeY && y < newSizeY; y++) {
                for (TCoord x = 0; x < sizeX && x < newSizeX; x++) {
                    newFirstBuffer[y * newSizeX + x] = firstBuffer[y * sizeX + x];
                }
            }
            sizeX = newSizeX;
            sizeY = newSizeY;
            firstBuffer = std::move(newFirstBuffer);
            secondBuffer = std::move(newSecondBuffer);
        }

        void setCenter(TCoord centerX, TCoord centerY) {
            auto deltaX = centerX - (offsetX + sizeX / 2);
            auto deltaY = centerY - (offsetY + sizeY / 2);
            if (deltaX | deltaY) {
                translate(deltaX, deltaY);
            }
        }

        void clear() {
            for (TCoord y = 0; y < sizeY; y++) {
                for (TCoord x = 0; x < sizeX; x++) {
                    auto i = y * sizeX + x;
                    auto value = firstBuffer[i];
                    firstBuffer[i] = {};
                    if (outCallback && value != T {}) {
                        outCallback(x + offsetX, y + offsetY, value);
                    }
                }
            }
            valuesCount = 0;
        }

        TCoord getOffsetX() const {
            return offsetX;
        }

        TCoord getOffsetY() const {
            return offsetY;
        }

        TCoord getWidth() const {
            return sizeX;
        }

        TCoord getHeight() const {
            return sizeY;
        }

        const std::vector<T>& getBuffer() const {
            return firstBuffer;
        }

        size_t count() const {
            return valuesCount;
        }

        TCoord area() const {
            return sizeX * sizeY;
        }
    };
}
