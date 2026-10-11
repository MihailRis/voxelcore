#ifndef VOXELCOREAPI_H
#define VOXELCOREAPI_H
#ifdef __cplusplus
extern "C" {
#endif

typedef struct Mainloop_t Mainloop_t;
typedef struct Engine_t Engine_t;
typedef struct CoreParameters_t CoreParameters_t;

Mainloop_t* mainloop_create(Engine_t* engine);
void mainloop_run(Mainloop_t* mainloop);
void mainloop_free(Mainloop_t* mainloop);

CoreParameters_t* core_parameters_create(void);
void core_parameters_free(CoreParameters_t* params);
void core_parameters_set_headless(CoreParameters_t* params, int headless);
void core_parameters_set_test_mode(CoreParameters_t* params, int test_mode);
void core_parameters_set_stdin_commands(
    CoreParameters_t* params, int stdin_commands
);
void core_parameters_set_tps(CoreParameters_t* params, int tps);
void core_parameters_set_subprocess_depth(CoreParameters_t* params, int depth);

void core_parameters_set_res_folder(CoreParameters_t* params, const char* path);
void core_parameters_set_user_folder(
    CoreParameters_t* params, const char* path
);
void core_parameters_set_script_file(
    CoreParameters_t* params, const char* path
);
void core_parameters_set_project_folder(
    CoreParameters_t* params, const char* path
);
void core_parameters_set_log_file(CoreParameters_t* params, const char* path);
void core_parameters_set_debug_server_string(
    CoreParameters_t* params, const char* str
);

void core_parameters_add_project_arg(
    CoreParameters_t* params, const char* key, const char* value
);

Engine_t* engine_get_instance(void);
void engine_init(Engine_t* engine, CoreParameters_t* params);
void engine_run(Engine_t* engine);
void engine_close(Engine_t* engine);
void engine_terminate(void);
void engine_quit(Engine_t* engine);
int engine_is_quit_signal(const Engine_t* engine);
int engine_is_headless(const Engine_t* engine);

#ifdef __cplusplus
}
#endif
#endif /** VOXELCOREAPI_H */