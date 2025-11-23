#pragma once

#include <memory>
#include <vector>
#include <optional>
#include <string>
#include <istream>
#include <ostream>

#include "io/io.hpp"

namespace scripting {

    struct StreamDescriptor {
        std::unique_ptr<std::istream> in;
        std::unique_ptr<std::ostream> out;
    };

    class descriptors_manager {
    private:
        static std::vector<std::optional<StreamDescriptor>> descriptors;

    public:
        static std::istream* get_input(int descriptor);
        static std::ostream* get_output(int descriptor);

        static void flush(int descriptor);

        static bool has_descriptor(int descriptor);

        static bool is_readable(int descriptor);
        static bool is_writeable(int descriptor);

        static void close(int descriptor);
        static int open_descriptor(const io::path& path, bool write, bool read);

        static void close_all_descriptors();
    };
}