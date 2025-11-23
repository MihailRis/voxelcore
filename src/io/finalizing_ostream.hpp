#pragma once

#include <memory>
#include <ostream>
#include <functional>

class finalizing_ostream final : public std::ostream {
public:
    finalizing_ostream(
        std::unique_ptr<std::ostream> inner,
        std::function<void(std::unique_ptr<std::ostream>)> on_destruction
    )
        : std::ostream(inner->rdbuf()),
          innerStream(std::move(inner)),
          onDestruction(on_destruction) {
    }

    finalizing_ostream(const finalizing_ostream&) = delete;
    finalizing_ostream& operator=(const finalizing_ostream&) = delete;

    finalizing_ostream(finalizing_ostream&& other) noexcept
        : std::ostream(std::move(other)),
          innerStream(std::move(other.innerStream)),
          onDestruction(std::move(other.onDestruction)) {
        other.onDestruction = nullptr;
    }

    finalizing_ostream& operator=(finalizing_ostream&& other) noexcept {
        if (this != &other) {
            std::ostream::operator=(std::move(other));
            innerStream = std::move(other.innerStream);
            onDestruction = std::move(other.onDestruction);
            other.onDestruction = nullptr;
        }
        return *this;
    }

    ~finalizing_ostream() {
        if (onDestruction) {
            onDestruction(std::move(innerStream));
        }
    }

private:
    std::unique_ptr<std::ostream> innerStream;
    std::function<void(std::unique_ptr<std::ostream>)> onDestruction;
};
