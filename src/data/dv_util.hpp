#pragma once

#include "dv.hpp"

#include <glm/glm.hpp>

namespace dv {
    template <int n, typename T>
    dv::value to_value(const glm::vec<n, T>& vec) {
        auto list = dv::list();
        for (size_t i = 0; i < n; i++) {
            list.add(vec[i]);
        }
        return list;
    }

    template <int n, int m, typename T>
    dv::value to_value(const glm::mat<n, m, T>& mat) {
        auto list = dv::list();
        for (size_t i = 0; i < n; i++) {
            for (size_t j = 0; j < m; j++) {
                list.add(mat[i][j]);
            }
        }
        return list;
    }

    template <typename T>
    dv::value to_value(const std::vector<T>& vec) {
        auto values = dv::list();
        for (const auto& value : vec) {
            values.add(value);
        }
        return values;
    }

    template <int n>
    void get_vec(const dv::value& list, glm::vec<n, float>& vec) {
        for (size_t i = 0; i < n; i++) {
            vec[i] = list[i].asNumber();
        }
    }

    template <int n, typename T>
    void get_vec(const dv::value& map, const std::string& key, glm::vec<n, T>& vec) {
        if (!map.has(key)) {
            return;
        }
        const auto& srcList = map[key];
        for (size_t i = 0; i < n; i++) {
            if constexpr (std::is_floating_point<T>()) {
                vec[i] = srcList[i].asNumber();
            } else {
                vec[i] = srcList[i].asInteger();
            }
        }
    }

    template <int n, int m>
    void get_mat(
        const dv::value& list,
        glm::mat<n, m, float>& mat
    ) {
        for (size_t y = 0; y < n; y++) {
            for (size_t x = 0; x < m; x++) {
                mat[y][x] = list[y * m + x].asNumber();
            }
        }
    }

    template <int n, int m>
    void get_mat(
        const dv::value& map,
        const std::string& key,
        glm::mat<n, m, float>& mat
    ) {
        if (!map.has(key)) {
            return;
        }
        auto& arr = map[key];
        for (size_t y = 0; y < n; y++) {
            for (size_t x = 0; x < m; x++) {
                mat[y][x] = arr[y * m + x].asNumber();
            }
        }
    }

    template <typename T>
    void get(const dv::value& map, const std::string& key, std::vector<T>& vec) {
        if (!map.has(key)) {
            return;
        }
        const auto& arr = map[key];
        for (size_t i = 0; i < arr.size(); i++) {
            T value;
            arr.at(i).get(value);
            vec.emplace_back(std::move(value));
        }
    }
}
