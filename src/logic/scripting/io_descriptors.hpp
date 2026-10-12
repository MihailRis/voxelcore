#pragma once

#include <istream>
#include <memory>
#include <ostream>

#include "io/fwd.hpp"

namespace scripting::io_descriptors {
    std::istream* get_input(int id);
    std::ostream* get_output(int id);

    std::istream& require_input(int id);
    std::ostream& require_output(int id);

    void flush(int id);
    int available(int id);

    bool has_descriptor(int id);

    bool is_readable(int id);
    bool is_writeable(int id);

    void close(int id);
    int open_descriptor(const io::path& path, bool write, bool read);
    int add_descriptor(std::unique_ptr<std::istream> stream);
    int add_descriptor(std::unique_ptr<std::ostream> stream);

    void close_all_descriptors();
}
