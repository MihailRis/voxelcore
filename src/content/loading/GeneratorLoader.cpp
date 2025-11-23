#define VC_ENABLE_REFLECTION
#include "../ContentLoader.hpp"

#include <algorithm>

#include "../ContentPack.hpp"

#include "io/io.hpp"
#include "engine/EnginePaths.hpp"
#include "logic/scripting/scripting.hpp"
#include "util/stringutil.hpp"
#include "world/generator/GeneratorDef.hpp"
#include "world/generator/VoxelFragment.hpp"
#include "debug/Logger.hpp"
#include "util/stringutil.hpp"

static BlocksLayer load_layer(
    const dv::value& map, uint& lastLayersHeight, bool& hasResizeableLayer
) {
    const auto& name = map["block"].asString();
    int height = map["height"].asInteger();
    bool belowSeaLevel = true;
    map.at("below-sea-level").get(belowSeaLevel);

    if (hasResizeableLayer) {
        lastLayersHeight += height;
    }
    if (height == -1) {
        if (hasResizeableLayer) {
            throw std::runtime_error("only one resizeable layer allowed");
        }
        hasResizeableLayer = true;
    }
    return BlocksLayer {name, height, belowSeaLevel, {}};
}

static inline BlocksLayers load_layers(
    const dv::value& layersArr, const std::string& fieldname
) {
    uint lastLayersHeight = 0;
    bool hasResizeableLayer = false;
    std::vector<BlocksLayer> layers;

    for (int i = 0; i < layersArr.size(); i++) {
        const auto& layerMap = layersArr[i];
        try {
            layers.push_back(
                load_layer(layerMap, lastLayersHeight, hasResizeableLayer));
        } catch (const std::runtime_error& err) {
            throw std::runtime_error(
                fieldname+" #"+std::to_string(i)+": "+err.what());
        }
    }
    return BlocksLayers {std::move(layers), lastLayersHeight};
}

static inline BiomeElementList load_biome_element_list(
    const dv::value map,
    const std::string& chanceName,
    const std::string& arrName,
    const std::string& nameName
) {
    float chance = 0.0f;
    map.at(chanceName).get(chance);
    std::vector<WeightedEntry> entries;
    if (map.has(arrName)) {
        const auto& arr = map[arrName];
        for (const auto& entry : arr) {
            const auto& name = entry[nameName].asString();
            float weight = entry["weight"].asNumber();
            if (weight <= 0.0f) {
                throw std::runtime_error("weight must be positive");
            }
            entries.push_back(WeightedEntry {name, weight, {}});
        }
    }
    std::sort(entries.begin(), entries.end(), std::greater<WeightedEntry>());
    return BiomeElementList(std::move(entries), chance);
}

static inline BiomeElementList load_plants(const dv::value& biomeMap) {
    return load_biome_element_list(biomeMap, "plant-chance", "plants", "block");
}

static inline BiomeElementList load_structures(const dv::value map) {
    return load_biome_element_list(map, "structure-chance", "structures", "name");
}

static debug::Logger logger("generator-loader");

static inline Biome load_biome(
    const dv::value& biomeMap,
    const std::string& name,
    uint parametersCount
) {
    std::vector<BiomeParameter> parameters;

    const auto& paramsArr = biomeMap["parameters"];
    if (paramsArr.size() < parametersCount) {
        throw std::runtime_error(
            std::to_string(parametersCount)+" parameters expected");
    }
    for (size_t i = 0; i < parametersCount; i++) {
        const auto& paramMap = paramsArr[i];
        float value = paramMap["value"].asNumber();
        float weight = paramMap["weight"].asNumber();
        parameters.push_back(BiomeParameter {value, weight});
    }

    auto plants = load_plants(biomeMap);
    auto groundLayers = load_layers(biomeMap["layers"], "layers");
    auto seaLayers = load_layers(biomeMap["sea-layers"], "sea-layers");
    
    BiomeElementList structures;
    if (biomeMap.has("structures")) {
        structures = load_structures(biomeMap);
    }
    return Biome {
        name,
        std::move(parameters),
        std::move(plants),
        std::move(structures),
        std::move(groundLayers),
        std::move(seaLayers)};
}

static VoxelStructureMeta load_structure_meta(
    const std::string& name, const dv::value& config
) {
    VoxelStructureMeta meta;
    meta.name = name;
    config.at("lowering").get(meta.lowering);
    return meta;
}

static std::vector<std::unique_ptr<VoxelStructure>> load_structures(
    const dv::value& map, const io::path& filesFolder, const ResPaths& paths
) {
    auto structuresDir = filesFolder / "fragments";

    std::vector<std::unique_ptr<VoxelStructure>> structures;
    for (auto& [name, config] : map.asObject()) {
        auto structFile = structuresDir / (name + ".vox");
        structFile = paths.find(structFile.string());
        logger.debug() << "loading voxel fragment " << structFile.string();
        if (!io::exists(structFile)) {
            throw std::runtime_error("structure file does not exist (" +
                structFile.string());
        }
        auto fragment = std::make_unique<VoxelFragment>();
        fragment->deserialize(io::read_binary_json(structFile));
        logger.info() << "fragment " << name << " has size [" << 
            fragment->getSize().x << ", " << fragment->getSize().y << ", " <<
            fragment->getSize().z << "]";

        structures.push_back(std::make_unique<VoxelStructure>(
            load_structure_meta(name, config),
            std::move(fragment)
        ));
    }
    return structures;
}

static void load_structures(
    GeneratorDef& def,
    const dv::value& map,
    const io::path& filesFolder,
    const ResPaths& paths
) {
    auto rawStructures = load_structures(map, filesFolder, paths);
    def.structures.resize(rawStructures.size());

    for (int i = 0; i < rawStructures.size(); i++) {
        def.structures[i] = std::move(rawStructures[i]);
    }
    // build indices map
    for (size_t i = 0; i < def.structures.size(); i++) {
        auto& structure = def.structures[i];
        def.structuresIndices[structure->meta.name] = i;
    }
}

static inline const io::path STRUCTURES_FILE = "structures.toml";
static inline const io::path BIOMES_FILE = "biomes.toml";
static inline const io::path GENERATORS_DIR = "generators";

static void load_biomes(GeneratorDef& def, const dv::value& root) {
    for (const auto& [biomeName, biomeMap] : root.asObject()) {
        try {
            def.biomes.push_back(
                load_biome(biomeMap, biomeName, def.biomeParameters));
        } catch (const std::runtime_error& err) {
            throw std::runtime_error("biome "+biomeName+": "+err.what());
        }
    }
}

void ContentLoader::loadGenerator(
    GeneratorDef& def, const std::string& full, const std::string& name
) {
    auto packDir = pack->folder;
    auto generatorsDir = packDir / GENERATORS_DIR;
    auto generatorFile = generatorsDir / (name + ".toml");
    if (!io::exists(generatorFile)) {
        return;
    }
    auto map = io::read_toml(generatorsDir / (name + ".toml"));
    map.at("caption").get(def.caption);
    map.at("biome-parameters").get(def.biomeParameters);
    map.at("biome-bpd").get(def.biomesBPD);
    map.at("heights-bpd").get(def.heightsBPD);
    std::string interpName;
    map.at("heights-interpolation").get(interpName);
    InterpolationTypeMeta.getItem(interpName, def.heightsInterpolation);
    map.at("biomes-interpolation").get(interpName);
    InterpolationTypeMeta.getItem(interpName, def.biomesInterpolation);

    map.at("sea-level").get(def.seaLevel);
    map.at("wide-structs-chunks-radius").get(def.wideStructsChunksRadius);
    if (map.has("heightmap-inputs")) {
        for (const auto& element : map["heightmap-inputs"]) {
            int index = element.asInteger();
            if (index <= 0 || index > def.biomeParameters) {
                throw std::runtime_error(
                    "invalid biome parameter index " + std::to_string(index));
            }
            def.heightmapInputs.push_back(index - 1);
        }
    }
    if (!def.heightmapInputs.empty() && def.biomesBPD != def.heightsBPD) {
        logger.warning() << "generator has heightmap-inputs but biomes-bpd "
            "is not equal to heights-bpd, generator will work slower!";
    }
    auto folder = generatorsDir / (name + ".files");
    auto scriptFile = folder / "script.lua";

    auto structuresFile = GENERATORS_DIR / (name + ".files") / STRUCTURES_FILE;
    auto structuresMap = paths.readCombinedObject(structuresFile.string());
    load_structures(def, structuresMap, structuresFile.parent(), paths);

    auto biomesFile = GENERATORS_DIR / (name + ".files") / BIOMES_FILE;
    auto biomesMap = paths.readCombinedObject(biomesFile.string(), true);
    if (biomesMap.empty()) {
        throw std::runtime_error(
            "generator " + util::quote(def.name) +
            ": at least one biome required"
        );
    }
    load_biomes(def, biomesMap);
    def.script = scripting::load_generator(
        def, scriptFile, pack->id+":generators/"+name+".files");
}
