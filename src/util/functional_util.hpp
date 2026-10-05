#pragma once

#include <glm/glm.hpp>

namespace util {
    template<class T, T (*F)(T, T)>
    struct binary_op {
        constexpr T operator()(T a, T b) const { return F(a, b); }
    };

    template<class T> using pow = binary_op<T, glm::pow>;
    template<class T> using min = binary_op<T, glm::min>;
    template<class T> using max = binary_op<T, glm::max>;


    template<class T, T (*F)(T)>
    struct unary_op {
        constexpr T operator()(T a) const { return F(a); }
    };

    template<typename T> using abs = unary_op<T, glm::abs>;
    template<typename T> using floor = unary_op<T, glm::floor>;
    template<typename T> using round = unary_op<T, glm::round>;
    template<typename T> using ceil = unary_op<T, glm::ceil>;
    template<typename T> using sin = unary_op<T, glm::sin>;
    template<typename T> using cos = unary_op<T, glm::cos>;
    template<typename T> using tan = unary_op<T, glm::tan>;
}
