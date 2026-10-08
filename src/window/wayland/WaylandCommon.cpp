#include "window/wayland/WaylandCommon.hpp"

#include <cstring>
#include <poll.h>

debug::Logger waylandLogger("window");


GlStateGuard::GlStateGuard() {
    glGetIntegerv(GL_CURRENT_PROGRAM, &values[0]);
    glGetIntegerv(GL_VERTEX_ARRAY_BINDING, &values[1]);
    glGetIntegerv(GL_ACTIVE_TEXTURE, &values[2]);
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &values[3]);
    glGetIntegerv(GL_UNPACK_ALIGNMENT, &values[4]);
    glGetIntegerv(GL_BLEND_SRC_RGB, &values[5]);
    glGetIntegerv(GL_BLEND_DST_RGB, &values[6]);
    glGetIntegerv(GL_BLEND_SRC_ALPHA, &values[7]);
    glGetIntegerv(GL_BLEND_DST_ALPHA, &values[8]);
    glGetIntegerv(GL_BLEND_EQUATION_RGB, &values[9]);
    glGetIntegerv(GL_BLEND_EQUATION_ALPHA, &values[10]);
    glGetIntegerv(GL_ARRAY_BUFFER_BINDING, &values[11]);
    glGetIntegerv(GL_DRAW_FRAMEBUFFER_BINDING, &values[12]);
    glGetIntegerv(GL_READ_FRAMEBUFFER_BINDING, &values[13]);
    glGetIntegerv(GL_DEPTH_WRITEMASK, &values[14]);
    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetIntegerv(GL_COLOR_WRITEMASK, colorMask);
    for (int i = 0; i < 3; i++) {
        attribFlags[i] = glIsEnabled(GL_VERTEX_ATTRIB_ARRAY_ENABLED + i);
        glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_SIZE, &attribs[i][0]);
        glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_TYPE, &attribs[i][1]);
        glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_NORMALIZED, &attribs[i][2]);
        glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_STRIDE, &attribs[i][3]);
        glGetVertexAttribiv(i, GL_VERTEX_ATTRIB_ARRAY_BUFFER_BINDING, &attribs[i][5]);
        void* pointer = nullptr;
        glGetVertexAttribPointerv(i, GL_VERTEX_ATTRIB_ARRAY_POINTER, &pointer);
        attribs[i][6] = static_cast<GLint>(reinterpret_cast<intptr_t>(pointer));
    }
    flags[0] = glIsEnabled(GL_BLEND);
    flags[1] = glIsEnabled(GL_DEPTH_TEST);
    flags[2] = glIsEnabled(GL_CULL_FACE);
    flags[3] = glIsEnabled(GL_SCISSOR_TEST);
}

GlStateGuard::~GlStateGuard() {
    glUseProgram(values[0]);
    glBindVertexArray(values[1]);
    glActiveTexture(values[2]);
    glBindTexture(GL_TEXTURE_2D, values[3]);
    glPixelStorei(GL_UNPACK_ALIGNMENT, values[4]);
    glBlendFuncSeparate(values[5], values[6], values[7], values[8]);
    glBlendEquationSeparate(values[9], values[10]);
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, values[12]);
    glBindFramebuffer(GL_READ_FRAMEBUFFER, values[13]);
    glDepthMask(values[14]);
    glColorMask(colorMask[0], colorMask[1], colorMask[2], colorMask[3]);
    glViewport(viewport[0], viewport[1], viewport[2], viewport[3]);
    const GLenum cap[] {GL_BLEND, GL_DEPTH_TEST, GL_CULL_FACE, GL_SCISSOR_TEST};
    for (int i = 0; i < 4; i++) {
        if (flags[i]) {
            glEnable(cap[i]);
        } else {
            glDisable(cap[i]);
        }
    }
    for (int i = 0; i < 3; i++) {
        glBindBuffer(GL_ARRAY_BUFFER, static_cast<GLuint>(attribs[i][5]));
        glVertexAttribPointer(
            i, attribs[i][0], static_cast<GLenum>(attribs[i][1]),
            attribs[i][2], attribs[i][3],
            reinterpret_cast<void*>(static_cast<intptr_t>(attribs[i][6]))
        );
        if (attribFlags[i]) {
            glEnableVertexAttribArray(i);
        } else {
            glDisableVertexAttribArray(i);
        }
    }
    glBindBuffer(GL_ARRAY_BUFFER, static_cast<GLuint>(values[11]));
}

GLuint compile_program(const char* vertexSource, const char* fragmentSource) {
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexSource, nullptr);
    glCompileShader(vertexShader);
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentSource, nullptr);
    glCompileShader(fragmentShader);
    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    return program;
}

WaylandState state;

class WaylandInput;
class WaylandWindow;

WaylandInput* input = nullptr;
WaylandWindow* window = nullptr;

bool decorations_enabled() {
    const char* value = getenv("VOXEL_DECORATIONS");
    return value == nullptr || strcmp(value, "none") != 0;
}

bool serverDecorations = false;

int keycode_from_keysym(xkb_keysym_t sym) {
    switch (sym) {
        case XKB_KEY_Escape: return GLFW_KEY_ESCAPE;
        case XKB_KEY_Return: return GLFW_KEY_ENTER;
        case XKB_KEY_Tab: return GLFW_KEY_TAB;
        case XKB_KEY_BackSpace: return GLFW_KEY_BACKSPACE;
        case XKB_KEY_Insert: return GLFW_KEY_INSERT;
        case XKB_KEY_Delete: return GLFW_KEY_DELETE;
        case XKB_KEY_Left: return GLFW_KEY_LEFT;
        case XKB_KEY_Right: return GLFW_KEY_RIGHT;
        case XKB_KEY_Up: return GLFW_KEY_UP;
        case XKB_KEY_Down: return GLFW_KEY_DOWN;
        case XKB_KEY_Page_Up: return GLFW_KEY_PAGE_UP;
        case XKB_KEY_Page_Down: return GLFW_KEY_PAGE_DOWN;
        case XKB_KEY_Home: return GLFW_KEY_HOME;
        case XKB_KEY_End: return GLFW_KEY_END;
        case XKB_KEY_Caps_Lock: return GLFW_KEY_CAPS_LOCK;
        case XKB_KEY_Scroll_Lock: return GLFW_KEY_SCROLL_LOCK;
        case XKB_KEY_Num_Lock: return GLFW_KEY_NUM_LOCK;
        case XKB_KEY_Print: return GLFW_KEY_PRINT_SCREEN;
        case XKB_KEY_Pause: return GLFW_KEY_PAUSE;
        case XKB_KEY_Menu: return GLFW_KEY_MENU;
        case XKB_KEY_Shift_L: return GLFW_KEY_LEFT_SHIFT;
        case XKB_KEY_Control_L: return GLFW_KEY_LEFT_CONTROL;
        case XKB_KEY_Alt_L: return GLFW_KEY_LEFT_ALT;
        case XKB_KEY_Super_L: return GLFW_KEY_LEFT_SUPER;
        case XKB_KEY_Shift_R: return GLFW_KEY_RIGHT_SHIFT;
        case XKB_KEY_Control_R: return GLFW_KEY_RIGHT_CONTROL;
        case XKB_KEY_Alt_R: return GLFW_KEY_RIGHT_ALT;
        case XKB_KEY_Super_R: return GLFW_KEY_RIGHT_SUPER;
        case XKB_KEY_space: return GLFW_KEY_SPACE;
        case XKB_KEY_apostrophe: return GLFW_KEY_APOSTROPHE;
        case XKB_KEY_comma: return GLFW_KEY_COMMA;
        case XKB_KEY_minus: return GLFW_KEY_MINUS;
        case XKB_KEY_period: return GLFW_KEY_PERIOD;
        case XKB_KEY_slash: return GLFW_KEY_SLASH;
        case XKB_KEY_semicolon: return GLFW_KEY_SEMICOLON;
        case XKB_KEY_equal: return GLFW_KEY_EQUAL;
        case XKB_KEY_bracketleft: return GLFW_KEY_LEFT_BRACKET;
        case XKB_KEY_backslash: return GLFW_KEY_BACKSLASH;
        case XKB_KEY_bracketright: return GLFW_KEY_RIGHT_BRACKET;
        case XKB_KEY_grave: return GLFW_KEY_GRAVE_ACCENT;
        case XKB_KEY_KP_0: return GLFW_KEY_KP_0;
        case XKB_KEY_KP_1: return GLFW_KEY_KP_1;
        case XKB_KEY_KP_2: return GLFW_KEY_KP_2;
        case XKB_KEY_KP_3: return GLFW_KEY_KP_3;
        case XKB_KEY_KP_4: return GLFW_KEY_KP_4;
        case XKB_KEY_KP_5: return GLFW_KEY_KP_5;
        case XKB_KEY_KP_6: return GLFW_KEY_KP_6;
        case XKB_KEY_KP_7: return GLFW_KEY_KP_7;
        case XKB_KEY_KP_8: return GLFW_KEY_KP_8;
        case XKB_KEY_KP_9: return GLFW_KEY_KP_9;
        case XKB_KEY_KP_Decimal: return GLFW_KEY_KP_DECIMAL;
        case XKB_KEY_KP_Divide: return GLFW_KEY_KP_DIVIDE;
        case XKB_KEY_KP_Multiply: return GLFW_KEY_KP_MULTIPLY;
        case XKB_KEY_KP_Subtract: return GLFW_KEY_KP_SUBTRACT;
        case XKB_KEY_KP_Add: return GLFW_KEY_KP_ADD;
        case XKB_KEY_KP_Enter: return GLFW_KEY_KP_ENTER;
        case XKB_KEY_KP_Equal: return GLFW_KEY_KP_EQUAL;
    }
    if (sym >= XKB_KEY_a && sym <= XKB_KEY_z) {
        return GLFW_KEY_A + (sym - XKB_KEY_a);
    }
    if (sym >= XKB_KEY_A && sym <= XKB_KEY_Z) {
        return GLFW_KEY_A + (sym - XKB_KEY_A);
    }
    if (sym >= XKB_KEY_0 && sym <= XKB_KEY_9) {
        return GLFW_KEY_0 + (sym - XKB_KEY_0);
    }
    if (sym >= XKB_KEY_F1 && sym <= XKB_KEY_F12) {
        return GLFW_KEY_F1 + (sym - XKB_KEY_F1);
    }
    return 0;
}

void dispatch_events(int timeoutMs) {
    wl_display_flush(state.display);
    while (wl_display_prepare_read(state.display) != 0) {
        wl_display_dispatch_pending(state.display);
        wl_display_flush(state.display);
    }
    pollfd descriptor {wl_display_get_fd(state.display), POLLIN, 0};
    if (poll(&descriptor, 1, timeoutMs) > 0) {
        if (wl_display_read_events(state.display) == -1) {
            wl_display_cancel_read(state.display);
        }
    } else {
        wl_display_cancel_read(state.display);
    }
    wl_display_dispatch_pending(state.display);
    if (wl_display_get_error(state.display) != 0) {
        on_display_error();
    }
}

