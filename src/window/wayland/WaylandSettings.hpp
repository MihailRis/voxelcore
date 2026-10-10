#pragma once

#include <string>

class SettingsWatcher {
public:
    SettingsWatcher();
    ~SettingsWatcher();

    bool isAvailable() const;

    bool read(const char* schema, const char* key, std::string& value);

    bool poll(const char* schema, const char* key, std::string& value);

private:
    bool open();
    void close();

    struct Impl;
    Impl* impl = nullptr;
};
