#pragma once

#include <glm/glm.hpp>
#include <string>

#include "data/dv_fwd.hpp"
#include "delegates.hpp"
#include "typedefs.hpp"

namespace scripting {
    using common_func = std::function<dv::value(const std::vector<dv::value>&)>;
    using value_to_string_func = std::function<std::string(const dv::value&)>;

    runnable create_runnable(
        const scriptenv& env,
        const std::string& src,
        const std::string& file = "[string]"
    );

    key_handler create_key_handler(
        const scriptenv& env,
        const std::string& src,
        const std::string& file = "[string]"
    );

    stringconsumer create_string_consumer(
        const scriptenv& env,
        const std::string& src,
        const std::string& file = "[string]"
    );

    wstringconsumer create_wstring_consumer(
        const scriptenv& env,
        const std::string& src,
        const std::string& file = "[string]"
    );

    wstringsupplier create_wstring_supplier(
        const scriptenv& env,
        const std::string& src,
        const std::string& file = "[string]"
    );

    wstringchecker create_wstring_validator(
        const scriptenv& env,
        const std::string& src,
        const std::string& file = "[string]"
    );

    boolconsumer create_bool_consumer(
        const scriptenv& env,
        const std::string& src,
        const std::string& file = "[string]"
    );

    boolsupplier create_bool_supplier(
        const scriptenv& env,
        const std::string& src,
        const std::string& file = "[string]"
    );

    doubleconsumer create_number_consumer(
        const scriptenv& env,
        const std::string& src,
        const std::string& file = "[string]"
    );

    doublesupplier create_number_supplier(
        const scriptenv& env,
        const std::string& src,
        const std::string& file = "[string]"
    );

    int_array_consumer create_int_array_consumer(
        const scriptenv& env,
        const std::string& src,
        const std::string& file = "[string]"
    );

    vec2supplier create_vec2_supplier(
        const scriptenv& env,
        const std::string& src,
        const std::string& file = "[string]"
    );

    value_to_string_func create_tostring(
        const scriptenv& env,
        const std::string& src,
        const std::string& file = "[string]"
    );
}
