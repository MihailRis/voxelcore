#include "logic/scripting/descriptors_manager.hpp"

#include "debug/Logger.hpp"

static debug::Logger logger("descriptors-manager");

namespace scripting {

    std::vector<std::optional<StreamDescriptor>> descriptors_manager::descriptors;

    std::istream* descriptors_manager::get_input(int descriptor) {
        if (!is_readable(descriptor))
            return nullptr;

        return descriptors[descriptor]->in.get();
    }

    std::ostream* descriptors_manager::get_output(int descriptor) {
        if (!is_writeable(descriptor))
            return nullptr;

        return descriptors[descriptor]->out.get();
    }

    void descriptors_manager::flush(int descriptor) {
        if (is_writeable(descriptor)) {
            descriptors[descriptor]->out->flush();
        }
    }

    bool descriptors_manager::has_descriptor(int descriptor) {
        return is_readable(descriptor) || is_writeable(descriptor);
    }

    bool descriptors_manager::is_readable(int descriptor) {
        return descriptor >= 0 && descriptor < static_cast<int>(descriptors.size())
            && descriptors[descriptor].has_value()
            && descriptors[descriptor]->in != nullptr;
    }

    bool descriptors_manager::is_writeable(int descriptor) {
        return descriptor >= 0 && descriptor < static_cast<int>(descriptors.size())
            && descriptors[descriptor].has_value()
            && descriptors[descriptor]->out != nullptr;
    }

    void descriptors_manager::close(int descriptor) {
        if (descriptor >= 0 && descriptor < static_cast<int>(descriptors.size())) {
            if (descriptors[descriptor].has_value()) {
                auto& desc = descriptors[descriptor].value();

                if (desc.out)
                    desc.out->flush();

                desc.in.reset();
                desc.out.reset();
            }

            descriptors[descriptor].reset();

            descriptors[descriptor] = std::nullopt;
        }
    }

    int descriptors_manager::open_descriptor(const io::path& path, bool write, bool read) {
        std::unique_ptr<std::istream> in;
        std::unique_ptr<std::ostream> out;

        try {
            if (read)
                in = io::read(path);

            if (write)
                out = io::write(path);
        } catch (const std::exception& e) {
            logger.error() << "failed to open descriptor for " << path.string()
                           << ": " << e.what();

            return -1;
        }

        for (int i = 0; i < static_cast<int>(descriptors.size()); ++i) {
            if (!descriptors[i].has_value()) {
                descriptors[i] = StreamDescriptor{ std::move(in), std::move(out) };
                return i;
            }
        }

        descriptors.emplace_back(StreamDescriptor{ std::move(in), std::move(out) });

        return static_cast<int>(descriptors.size() - 1);
    }


    void descriptors_manager::close_all_descriptors() {
        for (int i = 0; i < static_cast<int>(descriptors.size()); ++i) {
            if (descriptors[i].has_value()) {
                close(i);
            }
        }
        
        descriptors.clear();
    }
}