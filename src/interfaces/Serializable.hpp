#pragma once

#include "data/dv_fwd.hpp"

class Serializable {
public:
    virtual ~Serializable() {}
    virtual dv::value serialize() const = 0;
    virtual void deserialize(const dv::value& src) = 0;
};
