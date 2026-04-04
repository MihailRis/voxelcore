#include "WorldConverter.hpp"

#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>

#include "content/ContentReport.hpp"
#include "compatibility.hpp"
#include "debug/Logger.hpp"
#include "io/io.hpp"
#include "objects/Player.hpp"
#include "util/ThreadPool.hpp"
#include "voxels/Chunk.hpp"
#include "items/Inventory.hpp"
#include "voxels/Block.hpp"
#include "WorldFiles.hpp"

namespace fs = std::filesystem;

static debug::Logger logger("world-converter");

class ConverterWorker : public util::Worker<ConvertTask, int> {
    std::shared_ptr<WorldConverter> converter;
public:
    ConverterWorker(std::shared_ptr<WorldConverter> converter)
        : converter(std::move(converter)) {
    }

    int operator()(const ConvertTask& task) override {
        converter->convert(task);
        return 0;
    }
};

void WorldConverter::addRegionsTasks(
    RegionLayerIndex layerid,
    ConvertTaskType taskType
) {
    const auto& regions = wfile->getRegions();
    auto regionsFolder = regions.getRegionsFolder(layerid);
    if (!io::is_directory(regionsFolder)) {
        return;
    }
    for (const auto& file :io::directory_iterator(regionsFolder)) {
        int x, z;
        std::string name = file.stem();
        if (!WorldRegions::parseRegionFilename(name, x, z)) {
            logger.error() << "could not parse region name " << name;
            continue;
        }
        tasks.push(ConvertTask {taskType, file, x, z, layerid});
    }
}

void WorldConverter::createUpgradeTasks() {
    for (auto& issue : report->getIssues()) {
        if (issue.issueType != ContentIssueType::REGION_FORMAT_UPDATE) {
            continue;
        }
        addRegionsTasks(issue.regionLayer, ConvertTaskType::UPGRADE_REGION);
    }
}

void WorldConverter::createConvertTasks() {
    auto handleReorder = [=](ContentType contentType) {
        switch (contentType) {
            case ContentType::BLOCK:
                addRegionsTasks(
                    REGION_LAYER_VOXELS,
                    ConvertTaskType::VOXELS
                );
                break;
            case ContentType::ITEM:
                addRegionsTasks(
                    REGION_LAYER_INVENTORIES,
                    ConvertTaskType::INVENTORIES
                );
                break;
            default:
                break;
        }
    };

    for (auto& issue : report->getIssues()) {
        switch (issue.issueType) {
            case ContentIssueType::BLOCK_DATA_LAYOUTS_UPDATE:
            case ContentIssueType::REGION_FORMAT_UPDATE:
                break;
            case ContentIssueType::MISSING:
                throw std::runtime_error("issue can't be resolved");
            case ContentIssueType::REORDER:
                handleReorder(issue.contentType);
                break;
        }
    }

    tasks.push(ConvertTask {
        ConvertTaskType::PLAYER, wfile->getPlayerFile(), 0, 0, {}});
}

void WorldConverter::createBlockFieldsConvertTasks() {
    // blocks data conversion requires correct block indices
    // so it must be done AFTER voxels conversion
    for (auto& issue : report->getIssues()) {
        switch (issue.issueType) {
            case ContentIssueType::BLOCK_DATA_LAYOUTS_UPDATE:
                addRegionsTasks(
                    REGION_LAYER_BLOCKS_DATA,
                    ConvertTaskType::CONVERT_BLOCKS_DATA
                );
                break;
            default:
                break;
        }
    }
}

WorldConverter::WorldConverter(
    const std::shared_ptr<WorldFiles>& worldFiles,
    const Content* content,
    std::shared_ptr<ContentReport> reportPtr,
    ConvertMode mode
)
    : wfile(worldFiles),
      report(std::move(reportPtr)),
      content(content),
      mode(mode)
{
    switch (mode) {
        case ConvertMode::UPGRADE:
            createUpgradeTasks();
            break;
        case ConvertMode::REINDEX:
            createConvertTasks();
            break;
        case ConvertMode::BLOCK_FIELDS:
            createBlockFieldsConvertTasks();
            break;
    }
}

WorldConverter::~WorldConverter() {
}

std::shared_ptr<Task> WorldConverter::startTask(
    const std::shared_ptr<WorldFiles>& worldFiles,
    const Content* content,
    const std::shared_ptr<ContentReport>& report,
    const runnable& onDone,
    ConvertMode mode,
    bool multithreading
) {
    auto converter = std::make_shared<WorldConverter>(
        worldFiles, content, report, mode);
    if (!multithreading) {
        converter->setOnComplete([=]() {
            converter->write();
            onDone();
        });
        return converter;
    }
    auto pool = std::make_shared<util::ThreadPool<ConvertTask, int>>(
        "converter-pool",
        [=]() { return std::make_unique<ConverterWorker>(converter); },
        [=](int&&) {}
    );
    auto& converterTasks = converter->tasks;
    while (!converterTasks.empty()) {
        ConvertTask task = std::move(converterTasks.front());
        converterTasks.pop();
        pool->enqueueJob(std::move(task));
        
    }
    pool->setOnComplete([=]() {
        converter->write();
        onDone();
    });
    return pool;
}

void WorldConverter::upgradeRegion(
    const io::path& file, int x, int z, RegionLayerIndex layer
) const {
    auto path = wfile->getRegions().getRegionFilePath(layer, x, z);
    auto bytes = io::read_bytes_buffer(path);
    auto buffer = compatibility::convert_region_2to3(bytes, layer);
    io::write_bytes(path, buffer.data(), buffer.size());
}

void WorldConverter::convertVoxels(const io::path& file, int x, int z) const {
    logger.info() << "converting voxels region " << x << "_" << z;
    wfile->getRegions().processRegion(x, z, REGION_LAYER_VOXELS,
    [=](std::unique_ptr<ubyte[]> data, uint32_t*) {
        Chunk::convert(data.get(), report.get());
        return data;
    });
}

void WorldConverter::convertInventories(const io::path& file, int x, int z) const {
    logger.info() << "converting inventories region " << x << "_" << z;
    wfile->getRegions().processInventories(x, z, [=](Inventory* inventory) {
        inventory->convert(report.get());
    });
}

void WorldConverter::convertPlayer(const io::path& file) const {
    logger.info() << "converting player " << file.string();
    auto map = io::read_json(file);
    Player::convert(map, report.get());
    io::write_json(file, map);
}

void WorldConverter::convertBlocksData(int x, int z, const ContentReport& report) const {
    logger.info() << "converting blocks data";
    wfile->getRegions().processBlocksData(x, z, 
    [=](BlocksMetadata* heap, std::unique_ptr<ubyte[]> voxelsData) {
        Chunk chunk(0, 0);
        chunk.decode(voxelsData.get());

        const auto& indices = content->getIndices()->blocks;

        BlocksMetadata newHeap;
        for (const auto& entry : *heap) {
            size_t index = entry.index;
            const auto& def = indices.require(chunk.voxels[index].id);
            const auto& newStruct = *def.dataStruct;
            const auto& found = report.blocksDataLayouts.find(def.name);
            if (found == report.blocksDataLayouts.end()) {
                logger.error() << "no previous fields layout found for block" 
                    << def.name << " - discard";
                continue; 
            }
            const auto& prevStruct = found->second;
            uint8_t* dst = newHeap.allocate(index, newStruct.size());
            newStruct.convert(prevStruct, entry.data(), dst, true);
        }
        *heap = std::move(newHeap);
    });
}

void WorldConverter::convert(const ConvertTask& task) const {
    if (!io::is_regular_file(task.file)) return;

    switch (task.type) {
        case ConvertTaskType::UPGRADE_REGION:
            upgradeRegion(task.file, task.x, task.z, task.layer);
            break;
        case ConvertTaskType::VOXELS:
            convertVoxels(task.file, task.x, task.z);
            break;
        case ConvertTaskType::INVENTORIES:
            convertInventories(task.file, task.x, task.z);
            break;
        case ConvertTaskType::PLAYER:
            convertPlayer(task.file);
            break;
        case ConvertTaskType::CONVERT_BLOCKS_DATA:
            convertBlocksData(task.x, task.z, *report);
            break;
    }
}

void WorldConverter::convertNext() {
    if (tasks.empty()) {
        throw std::runtime_error("no more regions to convert");
    }
    ConvertTask task = tasks.front();
    tasks.pop();
    tasksDone++;

    convert(task);
}

void WorldConverter::setOnComplete(runnable callback) {
    this->onComplete = std::move(callback);
}

void WorldConverter::update() {
    convertNext();
    if (onComplete && tasks.empty()) {
        onComplete();
    }
}

void WorldConverter::terminate() {
    tasks = {};
}

bool WorldConverter::isActive() const {
    return !tasks.empty();
}

void WorldConverter::write() {
    logger.info() << "applying changes";

    auto patch = dv::object();
    switch (mode) {
        case ConvertMode::UPGRADE:
            patch["region-version"] = REGION_FORMAT_VERSION;
            break;
        case ConvertMode::REINDEX:
            WorldFiles::createContentIndicesCache(content->getIndices(), patch);
            break;
        case ConvertMode::BLOCK_FIELDS:
            WorldFiles::createBlockFieldsIndices(content->getIndices(), patch);
            break;
    }
    wfile->patchIndicesFile(patch);
    wfile->write(nullptr, nullptr);
}

void WorldConverter::waitForEnd() {
    while (isActive()) {
        update();
    }
}

uint WorldConverter::getWorkTotal() const {
    return tasks.size() + tasksDone;
}

uint WorldConverter::getWorkDone() const {
    return tasksDone;
}
