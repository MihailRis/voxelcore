#pragma once

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "delegates.hpp"
#include "typedefs.hpp"
#include "util/observer_handler.hpp"

enum class setting_format { simple, percent };

class Setting {
protected:
    setting_format format;
public:
    Setting(setting_format format) : format(format) {
    }

    virtual ~Setting() {
    }

    virtual void resetToDefault() = 0;

    virtual setting_format getFormat() const {
        return format;
    }

    virtual std::string toString() const = 0;
};

template <class T>
class ObservableSetting : public Setting {
    int nextid = 1;
    std::unordered_map<int, consumer<T>> observers;
protected:
    T initial;
    T value;
public:
    ObservableSetting(T value, setting_format format)
        : Setting(format), initial(value), value(value) {
    }

    ObserverHandler observe(consumer<T> callback, bool callOnStart = false) {
        const int id = nextid++;
        observers.emplace(id, callback);
        if (callOnStart) {
            callback(value);
        }
        return ObserverHandler([this, id]() {
            observers.erase(id);
        });
    }

    [[nodiscard]] const T& get() const {
        return value;
    }

    const T& getDefault() const {
        return initial;
    }

    T& operator*() {
        return value;
    }

    void notify(T value) {
        for (auto& entry : observers) {
            entry.second(value);
        }
    }

    void set(T value) {
        if (value == this->value) {
            return;
        }
        this->value = value;
        notify(value);
    }

    virtual void resetToDefault() override {
        set(initial);
    }
};

class NumberSetting : public ObservableSetting<number_t> {
protected:
    number_t min;
    number_t max;
public:
    NumberSetting(
        number_t value,
        number_t min = std::numeric_limits<number_t>::min(),
        number_t max = std::numeric_limits<number_t>::max(),
        setting_format format = setting_format::simple
    )
        : ObservableSetting(value, format), min(min), max(max) {
    }

    number_t& operator*() {
        return value;
    }

    number_t get() const {
        return value;
    }

    number_t getMin() const {
        return min;
    }

    number_t getMax() const {
        return max;
    }

    number_t getT() const {
        return (value - min) / (max - min);
    }

    virtual std::string toString() const override;

    static inline NumberSetting createPercent(number_t def) {
        return NumberSetting(def, 0.0, 1.0, setting_format::percent);
    }
};

class IntegerSetting : public ObservableSetting<integer_t> {
protected:
    integer_t min;
    integer_t max;
public:
    IntegerSetting(
        integer_t value,
        integer_t min = std::numeric_limits<integer_t>::min(),
        integer_t max = std::numeric_limits<integer_t>::max(),
        setting_format format = setting_format::simple
    )
        : ObservableSetting(value, format), min(min), max(max) {
    }

    integer_t getMin() const {
        return min;
    }

    integer_t getMax() const {
        return max;
    }

    integer_t getT() const {
        return (value - min) / (max - min);
    }

    virtual std::string toString() const override;
};

class FlagSetting : public ObservableSetting<bool> {
public:
    FlagSetting(bool value, setting_format format = setting_format::simple)
        : ObservableSetting(value, format) {
    }

    void toggle() {
        set(!get());
    }

    virtual std::string toString() const override;
};

class StringSetting : public ObservableSetting<std::string> {
public:
    StringSetting(
        std::string value, setting_format format = setting_format::simple
    )
        : ObservableSetting(value, format) {
    }

    virtual std::string toString() const override;
};
