#pragma once

#include <string>
#include <memory>

class Engine;

namespace devtools {
    class SyntaxProcessor;

    class Editor {
    public:
        Editor(Engine& engine);
        ~Editor();

        void loadTools();

        SyntaxProcessor& getSyntaxProcessor();
    private:
        Engine& engine;
        std::unique_ptr<SyntaxProcessor> syntaxProcessor;
    };
}
