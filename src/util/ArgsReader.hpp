#pragma once

#include <string>
#include <stdexcept>
#include <cstring>

namespace util {
    class ArgsReader {
        const char* last = "";
        char** argv;
        int argc;
        int pos = 0;
    public:
        ArgsReader(int argc, char** argv) : argv(argv), argc(argc) {
        }

        void skip() {
            pos++;
        }

        bool hasNext() const {
            return pos < argc && std::strlen(argv[pos]);
        }

        bool isKeywordArg() const {
            return last[0] == '-';
        }

        std::string next() {
            if (pos >= argc) {
                throw std::runtime_error("unexpected end");
            }
            last = argv[pos];
            return argv[pos++];
        }

        int nextInt() {
            auto text = next();
            try {
                return std::stoi(text);
            } catch (const std::exception& e) {
                throw std::runtime_error(e.what());
            }
        }
    };
}
