#pragma once

#include <string>
#include <functional>

#include "io/fwd.hpp"
#include "data/dv_fwd.hpp"

struct ContentPack;

template<typename T> class ContentUnitBuilder;

template <typename DefT>
class ContentUnitLoader {
public:
    ContentUnitLoader(
        const ContentPack& pack,
        ContentUnitBuilder<DefT>& builder,
        const std::string& defsDir,
        std::function<void(DefT&)> postFunc = nullptr
    )
        : pack(pack),
          builder(builder),
          defsDir(defsDir),
          postFunc(std::move(postFunc)) {
    }
    void loadUnit(DefT& def, const std::string& full, const std::string& name);
    void loadUnit(DefT& def, const std::string& name, const io::path& file);
    void loadDefs(const dv::value& root);
private:
    const ContentPack& pack;
    ContentUnitBuilder<DefT>& builder;
    std::string defsDir;
    std::function<void(DefT&)> postFunc;
};

void process_method(
    dv::value& properties,
    const std::string& method,
    const std::string& name,
    const dv::value& value
);
