#include <gtest/gtest.h>

#include "coders/commons.hpp"
#include "coders/GLSLExtension.hpp"

TEST(GLSLExtension, processing) {
    GLSLExtension glsl;
    glsl.addHeader("sum", 
        glsl.process("sum.glsl",
            "// sum function for glsl\n"
            "float sum(float a, float b) {\n"
            "    return a + b;\n"
            "}\n",
            true, {}
        )
    );
    try {
        auto processed = glsl.process("test.glsl",
            "in vec2 v_uv;\n"
            "uniform sampler2D u_screen;\n"
            "\n"
            "#include /* hell\no */  <   sum     >\n"
            "#param float p_intensity\n"
            "#param vec3 p_pos = [0, 0, 0]\n"
            "\n"
            "vec4 effect() {\n"
            "    vec4 color = texture(u_screen, v_uv);\n"
            "    return mix(color, 1.0 - color, p_intensity);\n"
            "}\n",
        false, {});
        std::cout << processed.code << std::endl;
    } catch (const parsing_error& err) {
        std::cerr << err.errorLog() << std::endl;
        throw;
    }
}
