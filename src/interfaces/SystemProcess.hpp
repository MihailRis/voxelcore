#pragma once

#include "interfaces/Process.hpp"

class SystemProcess : public Process {
public:
    virtual ~SystemProcess() override = default;

    virtual int getStdinDescriptor() const = 0;
    virtual int getStdoutDescriptor() const = 0;
};
