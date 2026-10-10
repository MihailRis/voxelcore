#include "window/wayland/WaylandSettings.hpp"

#include "window/wayland/WaylandCommon.hpp"

#include <cstring>

#ifdef VOXELCORE_SDBUS
#include <systemd/sd-bus.h>
#endif

namespace {
    constexpr const char* PORTAL_SERVICE = "org.freedesktop.portal.Desktop";
    constexpr const char* PORTAL_PATH = "/org/freedesktop/portal/desktop";
    constexpr const char* PORTAL_INTERFACE = "org.freedesktop.portal.Settings";

#ifdef VOXELCORE_SDBUS
    std::string readVariant(sd_bus_message* message) {
        const char* contents = nullptr;
        char type = 0;
        if (sd_bus_message_peek_type(message, &type, &contents) < 0) {
            return {};
        }
        if (type == 'v') {
            if (sd_bus_message_enter_container(message, 'v', nullptr) < 0) {
                return {};
            }
            const std::string result = readVariant(message);
            sd_bus_message_exit_container(message);
            return result;
        }
        if (type == 's') {
            const char* text = nullptr;
            if (sd_bus_message_read(message, "s", &text) < 0 ||
                text == nullptr) {
                return {};
            }
            return text;
        }
        return {};
    }
#endif
}

struct SettingsWatcher::Impl {
#ifdef VOXELCORE_SDBUS
    sd_bus* bus = nullptr;
    std::string changedKey;
    std::string changedValue;
    bool changed = false;

    static int onSettingChanged(
        sd_bus_message* message, void* userdata, sd_bus_error*
    ) {
        auto* self = static_cast<Impl*>(userdata);
        const char* schema = nullptr;
        const char* key = nullptr;
        if (sd_bus_message_read(message, "ss", &schema, &key) < 0) {
            return 0;
        }
        const std::string value = readVariant(message);
        if (key != nullptr && !value.empty()) {
            self->changedKey = key;
            self->changedValue = value;
            self->changed = true;
        }
        return 0;
    }
#endif
};

SettingsWatcher::SettingsWatcher() : impl(new Impl()) {
    open();
}

SettingsWatcher::~SettingsWatcher() {
    close();
    delete impl;
}

bool SettingsWatcher::open() {
#ifdef VOXELCORE_SDBUS
    if (impl->bus != nullptr) {
        return true;
    }
    if (sd_bus_default_user(&impl->bus) < 0) {
        impl->bus = nullptr;
        return false;
    }
    sd_bus_match_signal(
        impl->bus, nullptr, nullptr, PORTAL_PATH, PORTAL_INTERFACE,
        "SettingChanged", &Impl::onSettingChanged, impl
    );
    return true;
#else
    return false;
#endif
}

void SettingsWatcher::close() {
#ifdef VOXELCORE_SDBUS
    if (impl->bus != nullptr) {
        sd_bus_flush_close_unref(impl->bus);
        impl->bus = nullptr;
    }
#endif
}

bool SettingsWatcher::isAvailable() const {
#ifdef VOXELCORE_SDBUS
    return impl->bus != nullptr;
#else
    return false;
#endif
}

bool SettingsWatcher::read(
    const char* schema, const char* key, std::string& value
) {
#ifdef VOXELCORE_SDBUS
    if (impl->bus == nullptr) {
        return false;
    }
    sd_bus_error error = SD_BUS_ERROR_NULL;
    sd_bus_message* reply = nullptr;
    const int result = sd_bus_call_method(
        impl->bus, PORTAL_SERVICE, PORTAL_PATH, PORTAL_INTERFACE, "Read",
        &error, &reply, "ss", schema, key
    );
    bool success = false;
    if (result >= 0) {
        if (sd_bus_message_enter_container(reply, 'v', nullptr) >= 0) {
            const std::string text = readVariant(reply);
            if (!text.empty()) {
                value = text;
                success = true;
            }
            sd_bus_message_exit_container(reply);
        }
    }
    if (reply != nullptr) {
        sd_bus_message_unref(reply);
    }
    sd_bus_error_free(&error);
    return success;
#else
    return false;
#endif
}

bool SettingsWatcher::poll(
    const char* schema, const char* key, std::string& value
) {
#ifdef VOXELCORE_SDBUS
    if (impl->bus == nullptr) {
        open();
        return false;
    }
    sd_bus_message* message = nullptr;
    int result = 0;
    while ((result = sd_bus_process(impl->bus, &message)) > 0) {
        if (message != nullptr) {
            sd_bus_message_unref(message);
            message = nullptr;
        }
    }
    if (!impl->changed || impl->changedKey != key) {
        return false;
    }
    impl->changed = false;
    value = impl->changedValue;
    return true;
#else
    return false;
#endif
}
