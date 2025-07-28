#pragma once

#include <type_traits>

template <typename T>
struct remove_const_ref_if_primitive {
    using type = T;
};

template <typename T>
struct remove_const_ref_if_primitive<const T&> {
    using stripped_type = typename std::remove_const<typename std::remove_reference<T>::type>::type;
    using type = typename std::conditional<std::is_fundamental<stripped_type>::value, 
                                           stripped_type, 
                                           const T&>::type;
};

template <typename T>
struct remove_const_ref_if_primitive<const T> {
    using stripped_type = typename std::remove_const<T>::type;
    using type = typename std::conditional<std::is_fundamental<stripped_type>::value,
                                           stripped_type,
                                           const T>::type;
};

template <typename T>
using remove_const_ref_if_primitive_t = typename remove_const_ref_if_primitive<T>::type;
