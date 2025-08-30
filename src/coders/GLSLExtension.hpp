#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "io/io.hpp"
#include "graphics/core/PostEffect.hpp"

class ResPaths;

class GLSLExtension {
public:
    using ParamsMap = std::unordered_map<std::string, PostEffect::Param>;

    struct ProcessingResult {
        std::string code;
        ParamsMap params;
    };

    void setPaths(const ResPaths* paths);

    void define(const std::string& name, std::string value);
    void undefine(const std::string& name);
    void setDefined(const std::string& name, bool defined);
    void addHeader(const std::string& name, ProcessingResult header);

    const ProcessingResult& getHeader(const std::string& name) const;
    const std::string& getDefine(const std::string& name) const;

    const std::unordered_map<std::string, std::string>& getDefines() const;

    bool hasHeader(const std::string& name) const;
    bool hasDefine(const std::string& name) const;
    void loadHeader(const std::string& name);
    
    ProcessingResult process(
        const io::path& file,
        const std::string& source,
        bool header = false
    );

    static inline std::string VERSION = "330 core";
private:
    std::unordered_map<std::string, ProcessingResult> headers;
    std::unordered_map<std::string, std::string> defines;

    const ResPaths* paths = nullptr;
};
