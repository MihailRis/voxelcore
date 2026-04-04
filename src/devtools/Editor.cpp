#include "Editor.hpp"

#include "engine/Engine.hpp"
#include "engine/EnginePaths.hpp"
#include "coders/syntax_parser.hpp"
#include "SyntaxProcessor.hpp"

using namespace devtools;

Editor::Editor(Engine& engine)
    : engine(engine), syntaxProcessor(std::make_unique<SyntaxProcessor>()) {
}

Editor::~Editor() = default;

void Editor::loadTools() {
    const auto& paths = engine.getResPaths();
    auto files = paths.listdir("devtools/syntax");
    for (const auto& file : files) {
        auto config = io::read_object(file);
        auto syntax = std::make_unique<Syntax>();
        syntax->deserialize(config);
        syntaxProcessor->addSyntax(std::move(syntax));
    }
}

SyntaxProcessor& Editor::getSyntaxProcessor() {
    return *syntaxProcessor;
}
