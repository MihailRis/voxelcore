#define VC_ENABLE_REFLECTION
#include "World.hpp"

#include <glm/glm.hpp>
#include <memory>
#include <utility>

#include "content/Content.hpp"
#include "content/ContentReport.hpp"
#include "debug/Logger.hpp"
#include "world/files/WorldFiles.hpp"
#include "items/Inventories.hpp"
#include "objects/Entities.hpp"
#include "objects/Player.hpp"
#include "objects/Players.hpp"
#include "settings.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/GlobalChunks.hpp"
#include "world/generator/WorldGenerator.hpp"
#include "world/generator/GeneratorDef.hpp"
#include "Level.hpp"

static debug::Logger logger("world");
const float DAYIME_SPECIFIC_SPEED = 1.0f / 1440.0f;  // 1.0f/60.0f/24.0f;

world_load_error::world_load_error(const std::string& message)
    : std::runtime_error(message) {
}

World::World(
    WorldInfo info,
    const std::shared_ptr<WorldFiles>& worldFiles,
    const Content& content,
    const std::vector<ContentPack>& packs
) : info(std::move(info)),
    content(content),
    packs(packs),
    wfile(std::move(worldFiles)) {}

World::~World() {
}

void World::updateTimers(float delta) {
    info.daytime += delta * info.daytimeSpeed * DAYIME_SPECIFIC_SPEED;
    info.daytime = std::fmod(info.daytime, 1.0f);
    info.totalTime += delta;
}

void World::writeResources(const Content& content) {
    auto root = dv::object();
    for (size_t typeIndex = 0; typeIndex < RESOURCE_TYPES_COUNT; typeIndex++) {
        auto typeName = ResourceTypeMeta.getNameString(static_cast<ResourceType>(typeIndex));
        auto& list = root.list(typeName);
        auto& indices = content.resourceIndices[typeIndex];
        for (size_t i = 0; i < indices.size(); i++) {
            auto& map = list.object();
            map["name"] = indices.getName(i);
            auto data = indices.getSavedData(i);
            if (data != nullptr) {
                map["saved"] = data;
            }
        }
    }
    io::write_json(wfile->getResourcesFile(), root);
}

void World::write(Level* level) {
    level->chunks->saveAll();
    info.nextEntityId = level->entities->peekNextID();
    wfile->write(this, &content);

    auto playerFile = level->players->serialize();
    io::write_json(wfile->getPlayerFile(), playerFile);

    writeResources(content);
}

std::unique_ptr<Level> World::create(
    const std::string& name,
    const std::string& generator,
    const io::path& directory,
    uint64_t seed,
    EngineSettings& settings,
    const Content& content,
    const std::vector<ContentPack>& packs
) {
    WorldInfo info {};
    info.name = name;
    info.generator = generator;
    info.seed = seed;
    auto world = std::make_unique<World>(
        info,
        std::make_unique<WorldFiles>(directory, settings.debug),
        content,
        packs
    );
    if (name.empty()) {
        logger.info() << "created nameless world";
    } else {
        logger.info() << "created world '" << name << "' ("
                      << directory.string() << ")";
    }
    logger.info() << "world seed: " << seed << " generator: " << generator;
    return std::make_unique<Level>(std::move(world), content, settings);
}

std::unique_ptr<Level> World::load(
    const std::shared_ptr<WorldFiles>& worldFilesPtr,
    EngineSettings& settings,
    const Content& content,
    const std::vector<ContentPack>& packs
) {
    auto worldFiles = worldFilesPtr.get();
    auto info = worldFiles->readWorldInfo();
    if (!info.has_value()) {
        throw world_load_error("could not to find world.json");
    }
    info->isLoaded = true;
    
    logger.info() << "loading world " << info->name << " ("
                  << worldFilesPtr->getFolder().string() << ")";
    logger.info() << "world version: " << info->major << "." << info->minor
                  << " seed: " << info->seed
                  << " generator: " << info->generator;

    auto world = std::make_unique<World>(
        info.value(), std::move(worldFilesPtr), content, packs
    );
    auto& wfile = world->wfile;
    wfile->readResourcesData(content);

    auto level = std::make_unique<Level>(std::move(world), content, settings);

    io::path file = wfile->getPlayerFile();
    if (!io::is_regular_file(file)) {
        logger.warning() << "player.json does not exists";
        level->players->create();
    } else {
        auto playerRoot = io::read_json(file);
        level->players->deserialize(playerRoot);

        if (!playerRoot["players"].empty()) {
            if (!playerRoot["players"][0].has("id")) {
                level->getWorld()->getInfo().nextPlayerId++;
            }
        }
    }
    return level;
}

std::shared_ptr<ContentReport> World::checkIndices(
    const std::shared_ptr<WorldFiles>& worldFiles, const Content* content
) {
    io::path indicesFile = worldFiles->getIndicesFile();
    if (io::is_regular_file(indicesFile)) {
        return ContentReport::create(worldFiles, indicesFile, content);
    }
    return nullptr;
}

void World::setName(const std::string& name) {
    this->info.name = name;
}

void World::setGenerator(const std::string& generator) {
    this->info.generator = generator;
}

bool World::hasPack(const std::string& id) const {
    for (auto& pack : packs) {
        if (pack.id == id) return true;
    }
    return false;
}

void World::setSeed(uint64_t seed) {
    this->info.seed = seed;
}

std::string World::getName() const {
    return info.name;
}

uint64_t World::getSeed() const {
    return info.seed;
}

std::string World::getGenerator() const {
    return info.generator;
}

const std::vector<ContentPack>& World::getPacks() const {
    return packs;
}

void WorldInfo::deserialize(const dv::value& root) {
    name = root["name"].asString();
    generator = root["generator"].asString(generator);
    seed = root["seed"].asInteger(seed);

    if (root.has("version")) {
        auto& verobj = root["version"];
        major = verobj["major"].asInteger();
        minor = verobj["minor"].asInteger();
    }
    if (root.has("time")) {
        auto& timeobj = root["time"];
        daytime = timeobj["day-time"].asNumber();
        daytimeSpeed = timeobj["day-time-speed"].asNumber();
        totalTime = timeobj["total-time"].asNumber();
    }
    if (root.has("weather")) {
        fog = root["weather"]["fog"].asNumber();
    }
    nextInventoryId = root["next-inventory-id"].asInteger(2);
    nextEntityId = root["next-entity-id"].asInteger(1);
    root.at("next-player-id").get(nextPlayerId);
}

dv::value WorldInfo::serialize() const {
    auto root = dv::object();

    auto& versionobj = root.object("version");
    versionobj["major"] = ENGINE_VERSION_MAJOR;
    versionobj["minor"] = ENGINE_VERSION_MINOR;

    root["name"] = name;
    root["generator"] = generator;
    root["seed"] = seed;

    auto& timeobj = root.object("time");
    timeobj["day-time"] = daytime;
    timeobj["day-time-speed"] = daytimeSpeed;
    timeobj["total-time"] = totalTime;

    root["weather"] = dv::object();
    root["weather"]["fog"] = fog;

    root["next-inventory-id"] = nextInventoryId;
    root["next-entity-id"] = nextEntityId;
    root["next-player-id"] = nextPlayerId;
    return root;
}
