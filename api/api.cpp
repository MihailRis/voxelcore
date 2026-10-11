
#include "api.h"

#include "engine/CoreParameters.hpp"
#include "engine/Engine.hpp"
#include "engine/Mainloop.hpp"

extern "C" {
Mainloop_t* mainloop_create(Engine_t* engine) {
    if (!engine) {
        return nullptr;
    }
    Engine* cpp_engine = reinterpret_cast<Engine*>(engine);
    Mainloop* cpp_mainloop = new Mainloop(*cpp_engine);
    return reinterpret_cast<Mainloop_t*>(cpp_engine);
}

void mainloop_run(Mainloop_t* mainloop) {
    if (mainloop) {
        reinterpret_cast<Mainloop*>(mainloop)->run();
    }
}

void mainloop_free(Mainloop_t* mainloop) {
    if (mainloop) {
        delete reinterpret_cast<Mainloop*>(mainloop);
    }
}

CoreParameters_t* core_parameters_create(void) {
    return reinterpret_cast<CoreParameters_t*>(new CoreParameters());
}

void core_parameters_free(CoreParameters_t* params) {
    if (params) {
        delete reinterpret_cast<CoreParameters*>(params);
    }
}

void core_parameters_set_headless(CoreParameters_t* params, int headless) {
    if (params) {
        reinterpret_cast<CoreParameters*>(params)->headless = (headless != 0);
    }
}
void core_parameters_set_test_mode(CoreParameters_t* params, int test_mode) {
    if (params) {
        reinterpret_cast<CoreParameters*>(params)->testMode = (test_mode != 0);
    }
}
void core_parameters_set_stdin_commands(
    CoreParameters_t* params, int stdin_commands
) {
    if (params) {
        reinterpret_cast<CoreParameters*>(params)->stdinCommands =
            (stdin_commands != 0);
    }
}
void core_parameters_set_tps(CoreParameters_t* params, int tps) {
    if (params) {
        reinterpret_cast<CoreParameters*>(params)->tps = tps;
    }
}
void core_parameters_set_subprocess_depth(CoreParameters_t* params, int depth) {
    if (params)
        reinterpret_cast<CoreParameters*>(params)->subProcessDepth = depth;
}

void core_parameters_set_res_folder(
    CoreParameters_t* params, const char* path
) {
    if (params && path)
        reinterpret_cast<CoreParameters*>(params)->resFolder = path;
}

void core_parameters_set_user_folder(
    CoreParameters_t* params, const char* path
) {
    if (params && path)
        reinterpret_cast<CoreParameters*>(params)->userFolder = path;
}
void core_parameters_set_script_file(
    CoreParameters_t* params, const char* path
) {
    if (params && path)
        reinterpret_cast<CoreParameters*>(params)->scriptFile = path;
}
void core_parameters_set_project_folder(
    CoreParameters_t* params, const char* path
) {
    if (params && path)
        reinterpret_cast<CoreParameters*>(params)->projectFolder = path;
}
void core_parameters_set_log_file(CoreParameters_t* params, const char* path) {
    if (params && path)
        reinterpret_cast<CoreParameters*>(params)->logFile = path;
}
void core_parameters_set_debug_server_string(
    CoreParameters_t* params, const char* str
) {
    if (params && str)
        reinterpret_cast<CoreParameters*>(params)->debugServerString = str;
}

void core_parameters_add_project_arg(
    CoreParameters_t* params, const char* key, const char* value
) {
    if (params && key && value) {
        reinterpret_cast<CoreParameters*>(params)->projectArgs[key] = value;
    }
}

Engine_t* engine_get_instance(void) {
    return reinterpret_cast<Engine_t*>(&Engine::getInstance());
}

void engine_init(Engine_t* engine, CoreParameters_t* params) {
    if (engine && params) {
        Engine* voxel_engine = reinterpret_cast<Engine*>(engine);
        CoreParameters* voxel_params =
            reinterpret_cast<CoreParameters*>(params);
        voxel_engine->initialize(*voxel_params);
    }
}

void engine_run(Engine_t* engine) {
    if (engine) {
        reinterpret_cast<Engine*>(engine)->run();
    }
}

void engine_close(Engine_t* engine) {
    if (engine) {
        reinterpret_cast<Engine*>(engine)->close();
    }
}

void engine_terminate(void) {
    Engine::terminate();
}

void engine_quit(Engine_t* engine) {
    if (engine) {
        reinterpret_cast<Engine*>(engine)->quit();
    }
}

int engine_is_quit_signal(const Engine_t* engine) {
    if (engine) {
        return reinterpret_cast<const Engine*>(engine)->isQuitSignal() ? 1 : 0;
    }
    return 0;
}

int engine_is_headless(const Engine_t* engine) {
    if (engine) {
        return reinterpret_cast<const Engine*>(engine)->isHeadless() ? 1 : 0;
    }
    return 0;
}
}
