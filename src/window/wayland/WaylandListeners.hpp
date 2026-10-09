#pragma once

#include "window/wayland/WaylandCommon.hpp"

void toplevel_configure(
    void*, xdg_toplevel*, int32_t width, int32_t height, wl_array* states
);
void toplevel_close(void*, xdg_toplevel*);
void toplevel_configure_bounds(void*, xdg_toplevel*, int32_t, int32_t);
void toplevel_wm_capabilities(void*, xdg_toplevel*, wl_array*);
void xdg_surface_configure(void*, xdg_surface* surface, uint32_t serial);
void data_device_data_offer(void*, wl_data_device*, wl_data_offer* offer);
void data_device_selection(void*, wl_data_device*, wl_data_offer* offer);
void registry_global(
    void*, wl_registry* registry, uint32_t name, const char* interface,
    uint32_t version
);
void registry_global_remove(void*, wl_registry*, uint32_t);
void zxdg_decoration_configure(
    void*, zxdg_toplevel_decoration_v1*, uint32_t mode
);
