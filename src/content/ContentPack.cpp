#include "ContentPack.hpp"

#include <algorithm>
#include <iostream>
#include <utility>

#include "coders/json.hpp"
#include "constants.hpp"
#include "data/dv.hpp"
#include "io/engine_paths.hpp"
#include "io/io.hpp"


namespace fs = std::filesystem;

ContentPack ContentPack::createCore() {
    return ContentPack {
        "core", "Core", ENGINE_VERSION_STRING, "", "", "res:", {}
    };
}

const std::vector<std::string> ContentPack::RESERVED_NAMES = {
    "res", "abs", "local", "core", "user", "world", "none", "null", "project", 
    "pack", "packid", "root"
};

contentpack_error::contentpack_error(
    std::string packId, io::path folder, const std::string& message
)
    : std::runtime_error(message),
      packId(std::move(packId)),
      folder(std::move(folder)) {
}

std::string contentpack_error::getPackId() const {
    return packId;
}
io::path contentpack_error::getFolder() const {
    return folder;
}

io::path ContentPack::getContentFile() const {
    return folder / CONTENT_FILENAME;
}

bool ContentPack::is_pack(const io::path& folder) {
    return io::is_regular_file(folder / PACKAGE_FILENAME);
}

static void checkContentPackId(const std::string& id, const io::path& folder) {
    if (id.length() < 2 || id.length() > 24)
        throw contentpack_error(
            id, folder, "content-pack id length is out of range [2, 24]"
        );
    if (isdigit(id[0]))
        throw contentpack_error(
            id, folder, "content-pack id must not start with a digit"
        );
    for (char c : id) {
        if (!isalnum(c) && c != '_') {
            throw contentpack_error(
                id, folder, "illegal character in content-pack id"
            );
        }
    }
    if (std::find(
            ContentPack::RESERVED_NAMES.begin(),
            ContentPack::RESERVED_NAMES.end(),
            id
        ) != ContentPack::RESERVED_NAMES.end()) {
        throw contentpack_error(id, folder, "this content-pack id is reserved");
    }
}

ContentPack ContentPack::read(const io::path& folder) {
    auto root = io::read_json(folder / PACKAGE_FILENAME);
    ContentPack pack;
    root.at("id").get(pack.id);
    root.at("title").get(pack.title);
    root.at("version").get(pack.version);
    if (root.has("creators")) {
        const auto& creators = root["creators"];
        for (int i = 0; i < creators.size(); i++) {
            if (i > 0) {
                pack.creator += ", ";
            }
            pack.creator += creators[i].asString();
        }
    } else {
        root.at("creator").get(pack.creator);
    }
    root.at("description").get(pack.description);
    root.at("source").get(pack.source);
    pack.folder = folder;

    if (auto found = root.at("dependencies")) {
        const auto& dependencies = *found;
        for (const auto& elem : dependencies) {
            std::string depName = elem.asString();
            auto level = DependencyLevel::required;
            switch (depName.at(0)) {
                case '!':
                    depName = depName.substr(1);
                    break;
                case '?':
                    depName = depName.substr(1);
                    level = DependencyLevel::optional;
                    break;
                case '~':
                    depName = depName.substr(1);
                    level = DependencyLevel::weak;
                    break;
            }
            pack.dependencies.push_back({level, depName});
        }
    }

    if (pack.id == "none") {
        throw contentpack_error(
            pack.id, folder, "content-pack id is not specified"
        );
    }
    checkContentPackId(pack.id, folder);

    return pack;
}

void ContentPack::scanFolder(
    const io::path& folder, std::vector<ContentPack>& packs
) {
    if (!io::is_directory(folder)) {
        return;
    }
    for (const auto& packFolder : io::directory_iterator(folder)) {
        if (!io::is_directory(packFolder)) continue;
        if (!is_pack(packFolder)) continue;
        try {
            packs.push_back(read(packFolder));
        } catch (const contentpack_error& err) {
            std::cerr << "package.json error at " << err.getFolder().string();
            std::cerr << ": " << err.what() << std::endl;
        } catch (const std::runtime_error& err) {
            std::cerr << err.what() << std::endl;
        }
    }
}

std::vector<std::string> ContentPack::worldPacksList(const io::path& folder) {
    io::path listfile = folder / "packs.list";
    if (!io::is_regular_file(listfile)) {
        throw std::runtime_error("missing file 'packs.list'");
    }
    return io::read_list(listfile);
}

io::path ContentPack::findPack(
    const EnginePaths* paths, const io::path& worldDir, const std::string& name
) {
    io::path folder = worldDir / "content" / name;
    if (io::is_directory(folder)) {
        return folder;
    }
    folder = io::path("user:content") / name;
    if (io::is_directory(folder)) {
        return folder;
    }
    return io::path("res:content") / name;
}

ContentPackRuntime::ContentPackRuntime(ContentPack info, scriptenv env)
    : info(std::move(info)), env(std::move(env)) {
}

ContentPackRuntime::~ContentPackRuntime() = default;
