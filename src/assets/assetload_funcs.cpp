#include "assetload_funcs.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>

#include "audio/audio.hpp"
#include "coders/GLSLExtension.hpp"
#include "coders/commons.hpp"
#include "coders/imageio.hpp"
#include "coders/json.hpp"
#include "coders/obj.hpp"
#include "coders/vcm.hpp"
#include "coders/vec3.hpp"
#include "constants.hpp"
#include "debug/Logger.hpp"
#include "io/engine_paths.hpp"
#include "io/io.hpp"
#include "frontend/UiDocument.hpp"
#include "graphics/core/Atlas.hpp"
#include "graphics/core/Font.hpp"
#include "graphics/core/ImageData.hpp"
#include "graphics/core/Shader.hpp"
#include "graphics/core/Texture.hpp"
#include "graphics/core/TextureAnimation.hpp"
#include "graphics/commons/Model.hpp"
#include "objects/rigging.hpp"
#include "util/stringutil.hpp"
#include "Assets.hpp"
#include "AssetsLoader.hpp"

static debug::Logger logger("assetload-funcs");

namespace fs = std::filesystem;

static bool load_animation(
    Assets* assets,
    const ResPaths& paths,
    const std::string& atlasName,
    const std::string& directory,
    const std::string& name,
    Atlas* dstAtlas
);

assetload::postfunc assetload::texture(
    AssetsLoader*,
    const ResPaths& paths,
    const std::string& filename,
    const std::string& name,
    const std::shared_ptr<AssetCfg>&
) {
    auto actualFile = paths.find(filename + ".png");
    try {
        std::shared_ptr<ImageData> image(imageio::read(actualFile));
        return [name, image, actualFile](auto assets) {
            assets->store(Texture::from(image.get()), name);
        };
    } catch (const std::runtime_error& err) {
        logger.error() << actualFile.string() << ": " << err.what();
        return [](auto) {};
    }
}

static auto process_program(const ResPaths& paths, const std::string& filename) {
    io::path vertexFile = paths.find(filename + ".glslv");
    io::path fragmentFile = paths.find(filename + ".glslf");

    std::string vertexSource = io::read_string(vertexFile);
    std::string fragmentSource = io::read_string(fragmentFile);

    auto& preprocessor = *Shader::preprocessor;

    auto vertex = preprocessor.process(vertexFile, vertexSource);
    auto fragment = preprocessor.process(fragmentFile, fragmentSource);
    return std::make_pair(vertex, fragment);
}

assetload::postfunc assetload::shader(
    AssetsLoader*,
    const ResPaths& paths,
    const std::string& filename,
    const std::string& name,
    const std::shared_ptr<AssetCfg>&
) {
    auto [vertex, fragment] = process_program(paths, filename);

    io::path vertexFile = paths.find(filename + ".glslv");
    io::path fragmentFile = paths.find(filename + ".glslf");

    std::string vertexSource = std::move(vertex.code);
    std::string fragmentSource = std::move(fragment.code);

    return [=](auto assets) {
        assets->store(
            Shader::create(
                vertexFile.string(),
                fragmentFile.string(),
                vertexSource,
                fragmentSource
            ),
            name
        );
    };
}

assetload::postfunc assetload::posteffect(
    AssetsLoader*,
    const ResPaths& paths,
    const std::string& file,
    const std::string& name,
    const std::shared_ptr<AssetCfg>& settings
) {
    io::path effectFile = paths.find(file + ".glsl");
    std::string effectSource = io::read_string(effectFile);

    auto& preprocessor = *Shader::preprocessor;
    preprocessor.addHeader(
        "__effect__", preprocessor.process(effectFile, effectSource, true)
    );

    auto [vertex, fragment] = process_program(paths, SHADERS_FOLDER + "/effect");
    auto params = std::move(fragment.params);

    std::string vertexSource = std::move(vertex.code);
    std::string fragmentSource = std::move(fragment.code);

    return [=](auto assets) {
        auto program = Shader::create(
            effectFile.string(),
            effectFile.string(),
            vertexSource,
            fragmentSource
        );
        assets->store(
            std::make_shared<PostEffect>(std::move(program), params), name
        );
    };
}

static bool append_atlas(AtlasBuilder& atlas, const io::path& file) {
    std::string name = file.stem();
    // skip duplicates
    if (atlas.has(name)) {
        return false;
    }
    auto image = imageio::read(file);
    image->fixAlphaColor();
    atlas.add(name, std::move(image));
    return true;
}

assetload::postfunc assetload::atlas(
    AssetsLoader* loader,
    const ResPaths& paths,
    const std::string& directory,
    const std::string& name,
    const std::shared_ptr<AssetCfg>& config
) {
    auto atlasConfig = std::dynamic_pointer_cast<AtlasCfg>(config);
    if (atlasConfig && atlasConfig->type == AtlasType::SEPARATE) {
        for (const auto& file : paths.listdir(directory)) {
            if (!imageio::is_read_supported(file.extension()))
                continue;
            loader->add(
                AssetType::TEXTURE,
                directory + "/" + file.stem(),
                name + "/" + file.stem()
            );
        }
        return [](auto){};
    }
    AtlasBuilder builder;
    for (const auto& file : paths.listdir(directory)) {
        if (!imageio::is_read_supported(file.extension())) continue;
        if (!append_atlas(builder, file)) continue;
    }
    std::set<std::string> names = builder.getNames();
    Atlas* atlas = builder.build(2, false).release();
    return [=](auto assets) {
        atlas->prepare();
        assets->store(std::unique_ptr<Atlas>(atlas), name);
        for (const auto& file : names) {
            load_animation(assets, paths, name, directory, file, atlas);
        }
    };
}

assetload::postfunc assetload::font(
    AssetsLoader*,
    const ResPaths& paths,
    const std::string& filename,
    const std::string& name,
    const std::shared_ptr<AssetCfg>&
) {
    auto pages = std::make_shared<std::vector<std::unique_ptr<ImageData>>>();
    for (size_t i = 0; i <= 1024; i++) {
        std::string pagefile = filename + "_" + std::to_string(i) + ".png";
        auto file = paths.find(pagefile);
        if (io::exists(file)) {
            pages->push_back(imageio::read(file));
        } else if (i == 0) {
            throw std::runtime_error("font must have page 0");
        } else {
            pages->push_back(nullptr);
        }
    }
    return [=](auto assets) {
        int res = pages->at(0)->getHeight() / 16;
        std::vector<std::unique_ptr<Texture>> textures;
        for (auto& page : *pages) {
            if (page == nullptr) {
                textures.emplace_back(nullptr);
            } else {
                auto texture = Texture::from(page.get());
                texture->setMipMapping(false, true);
                textures.emplace_back(std::move(texture));
            }
        }
        assets->store(
            std::make_unique<Font>(std::move(textures), res, 4), name
        );
    };
}

assetload::postfunc assetload::layout(
    AssetsLoader*,
    const ResPaths&,
    const std::string& file,
    const std::string& name,
    const std::shared_ptr<AssetCfg>& config
) {
    return [=](auto assets) {
        try {
            auto cfg = std::dynamic_pointer_cast<LayoutCfg>(config);
            size_t pos = name.find(':');
            auto prefix = name.substr(0, pos);
            assets->store(
                UiDocument::read(
                    *cfg->gui,
                    cfg->env,
                    name,
                    file,
                    prefix + ":layouts/" + name.substr(pos + 1) + ".xml"
                ),
                name
            );
        } catch (const parsing_error& err) {
            throw std::runtime_error(
                "failed to parse layout XML '" + file + "':\n" + err.errorLog()
            );
        }
    };
}
assetload::postfunc assetload::sound(
    AssetsLoader*,
    const ResPaths& paths,
    const std::string& file,
    const std::string& name,
    const std::shared_ptr<AssetCfg>& config
) {
    auto cfg = std::dynamic_pointer_cast<SoundCfg>(config);
    bool keepPCM = cfg ? cfg->keepPCM : false;

    std::unique_ptr<audio::Sound> baseSound = nullptr;
    static std::vector<std::string> extensions {".ogg", ".wav"};
    std::string extension;
    for (size_t i = 0; i < extensions.size(); i++) {
        extension = extensions[i];
        // looking for 'sound_name' as base sound
        auto soundFile = paths.find(file + extension);
        if (io::exists(soundFile)) {
            baseSound = audio::load_sound(soundFile, keepPCM);
            break;
        }
        // looking for 'sound_name_0' as base sound
        auto variantFile = paths.find(file + "_0" + extension);
        if (io::exists(variantFile)) {
            baseSound = audio::load_sound(variantFile, keepPCM);
            break;
        }
    }
    if (baseSound == nullptr) {
        throw std::runtime_error("could not to find sound: " + file);
    }

    // loading sound variants
    for (uint i = 1;; i++) {
        auto variantFile =
            paths.find(file + "_" + std::to_string(i) + extension);
        if (!io::exists(variantFile)) {
            break;
        }
        baseSound->variants.emplace_back(audio::load_sound(variantFile, keepPCM));
    }

    auto sound = baseSound.release();
    return [=](auto assets) {
        assets->store(std::unique_ptr<audio::Sound>(sound), name);
    };
}

static void request_textures(AssetsLoader* loader, const model::Model& model) {
    for (auto& mesh : model.meshes) {
        if (mesh.texture.find('$') == std::string::npos &&
            mesh.texture.find(':') == std::string::npos) {
            auto filename = TEXTURES_FOLDER + "/" + mesh.texture;
            loader->add(
                AssetType::TEXTURE, filename, mesh.texture, nullptr
            );
        }
    }
}

assetload::postfunc assetload::model(
    AssetsLoader* loader,
    const ResPaths& paths,
    const std::string& file,
    const std::string& name,
    const std::shared_ptr<AssetCfg>&
) {
    auto path = paths.find(file + ".vec3");
    if (io::exists(path)) {
        auto bytes = io::read_bytes_buffer(path);
        auto modelVEC3 = std::make_shared<vec3::File>(vec3::load(path.string(), bytes));
        return [loader, name, modelVEC3=std::move(modelVEC3)](Assets* assets) {
            for (auto& [modelName, model] : modelVEC3->models) {
                request_textures(loader, model.model);
                std::string fullName = name;
                if (name != modelName) {
                    fullName += "." + modelName;
                }
                assets->store(
                    std::make_unique<model::Model>(model.model),
                    fullName
                );
                logger.info() << "store model " << util::quote(modelName)
                              << " as " << util::quote(fullName);
            }
        };
    }
    path = paths.find(file + ".obj");
    if (io::exists(path)) {
        auto text = io::read_string(path);
        try {
            auto model = obj::parse(path.string(), text).release();
            return [=](Assets* assets) {
                request_textures(loader, *model);
                assets->store(std::unique_ptr<model::Model>(model), name);
            };
        } catch (const parsing_error& err) {
            std::cerr << err.errorLog() << std::endl;
            throw;
        }
    }
    path = paths.find(file + ".xml");
    if (io::exists(path)) {
        auto text = io::read_string(path);
        try {
            auto model = vcm::parse(path.string(), text).release();
            return [=](Assets* assets) {
                request_textures(loader, *model);
                assets->store(std::unique_ptr<model::Model>(model), name);
            };
        } catch (const parsing_error& err) {
            std::cerr << err.errorLog() << std::endl;
            throw;
        }
    }
    throw std::runtime_error("could not to find model " + util::quote(file));
}

static void read_anim_file(
    const std::string& animFile,
    std::vector<std::pair<std::string, int>>& frameList
) {
    auto root = io::read_json(animFile);
    float frameDuration = DEFAULT_FRAME_DURATION;
    std::string frameName;

    if (auto found = root.at("frames")) {
        auto& frameArr = *found;
        for (size_t i = 0; i < frameArr.size(); i++) {
            const auto& currentFrame = frameArr[i];

            frameName = currentFrame[0].asString();
            if (currentFrame.size() > 1) {
                frameDuration = currentFrame[1].asNumber();
            }
            frameList.emplace_back(frameName, frameDuration);
        }
    }
}

static TextureAnimation create_animation(
    Atlas* srcAtlas,
    Atlas* dstAtlas,
    const std::string& name,
    const std::set<std::string>& frameNames,
    const std::vector<std::pair<std::string, int>>& frameList
) {
    Texture* srcTex = srcAtlas->getTexture();
    Texture* dstTex = dstAtlas->getTexture();
    UVRegion region = dstAtlas->get(name);

    TextureAnimation animation(srcTex, dstTex);
    Frame frame;

    uint dstWidth = dstTex->getWidth();
    uint dstHeight = dstTex->getHeight();

    uint srcWidth = srcTex->getWidth();
    uint srcHeight = srcTex->getHeight();

    const int extension = 2;

    frame.dstPos =
        glm::ivec2(region.u1 * dstWidth, region.v1 * dstHeight) - extension;
    frame.size = glm::ivec2(region.u2 * dstWidth, region.v2 * dstHeight) -
                 frame.dstPos + extension;

    for (const auto& elem : frameList) {
        if (!srcAtlas->has(elem.first)) {
            logger.error() << "unknown frame name: " << elem.first;
            continue;
        }
        region = srcAtlas->get(elem.first);
        if (elem.second > 0) {
            frame.duration = static_cast<float>(elem.second) / 1000.0f;
        }
        frame.srcPos = glm::ivec2(
            region.u1 * srcWidth, srcHeight - region.v2 * srcHeight
        ) - extension;
        animation.addFrame(frame);
    }
    return animation;
}

inline bool contains(
    const std::vector<std::pair<std::string, int>>& frameList,
    const std::string& frameName
) {
    for (const auto& elem : frameList) {
        if (frameName == elem.first) {
            return true;
        }
    }
    return false;
}

static bool load_animation(
    Assets* assets,
    const ResPaths& paths,
    const std::string& atlasName,
    const std::string& directory,
    const std::string& name,
    Atlas* dstAtlas
) {
    std::string animsDir = directory + "/animation";

    for (const auto& folder : paths.listdir(animsDir)) {
        if (!io::is_directory(folder)) continue;
        if (folder.name() != name) continue;
        //FIXME: if (fs::is_empty(folder)) continue;

        AtlasBuilder builder;
        append_atlas(builder, paths.find(directory + "/" + name + ".png"));

        std::vector<std::pair<std::string, int>> frameList;
        std::string animFile = folder.string() + "/animation.json";
        if (io::exists(animFile)) {
            read_anim_file(animFile, frameList);
        }
        for (const auto& file : paths.listdir(animsDir + "/" + name)) {
            if (!frameList.empty() &&
                !contains(frameList, file.stem())) {
                continue;
            }
            if (!append_atlas(builder, file)) continue;
        }
        auto srcAtlas = builder.build(2, true);
        if (frameList.empty()) {
            for (const auto& frameName : builder.getNames()) {
                frameList.emplace_back(frameName, 0);
            }
        }
        auto animation = create_animation(
            srcAtlas.get(), dstAtlas, name, builder.getNames(), frameList
        );
        assets->store(
            std::move(srcAtlas), atlasName + "/" + name + "_animation"
        );
        assets->store(animation);
        return true;
    }
    return true;
}
