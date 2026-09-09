#pragma once

#include <iostream>
#include <glm/glm.hpp>

template<int N, typename T>
std::ostream& operator<<(std::ostream& out, const glm::vec<N, T>& vec) {
    out << "{";
    for (int i = 0; i < N; i++) {
        if (i > 0) {
            out << ", ";
        }
        out << vec[i];
    }
    out << "}";
    return out;
}
