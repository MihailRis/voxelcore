#pragma once

#include <condition_variable>
#include <functional>
#include <glm/glm.hpp>
#include <memory>
#include <mutex>
#include <unordered_map>

#include "typedefs.hpp"
#include "util/BufferPool.hpp"
#include "voxels/Chunk.hpp"
#include "maths/voxmaths.hpp"
#include "coders/compression.hpp"
#include "io/io.hpp"
#include "world_regions_fwd.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/hash.hpp>

inline constexpr uint REGION_HEADER_SIZE = 10;

inline constexpr uint REGION_SIZE_BIT = 5;
inline constexpr uint REGION_SIZE = (1 << (REGION_SIZE_BIT));
inline constexpr uint REGION_CHUNKS_COUNT = ((REGION_SIZE) * (REGION_SIZE));

class illegal_region_format : public std::runtime_error {
public:
    illegal_region_format(const std::string& message)
        : std::runtime_error(message) {
    }
};

class WorldRegion {
    std::unique_ptr<std::unique_ptr<ubyte[]>[]> chunksData;
    std::unique_ptr<glm::u32vec2[]> sizes;
    bool unsaved = false;
public:
    WorldRegion();
    ~WorldRegion();

    void put(uint x, uint z, std::unique_ptr<ubyte[]> data, uint32_t size, uint32_t srcSize);
    ubyte* getChunkData(uint x, uint z);
    glm::u32vec2 getChunkDataSize(uint x, uint z);

    void setUnsaved(bool unsaved);
    bool isUnsaved() const;

    std::unique_ptr<ubyte[]>* getChunks() const;
    glm::u32vec2* getSizes() const;
};

struct regfile {
    io::rafile file;
    io::path filename;
    int version;
    bool inUse = false;

    regfile(io::path filename);
    regfile(const regfile&) = delete;

    std::unique_ptr<ubyte[]> read(int index, uint32_t& size, uint32_t& srcSize);
};

using RegionsMap = std::unordered_map<glm::ivec2, std::unique_ptr<WorldRegion>>;
using RegionProc = std::function<std::unique_ptr<ubyte[]>(std::unique_ptr<ubyte[]>,uint32_t*)>;
using InventoryProc = std::function<void(Inventory*)>;
using BlockDataProc = std::function<void(BlocksMetadata*, std::unique_ptr<ubyte[]>)>;

/// @brief Region file pointer keeping inUse flag on until destroyed
class regfile_ptr {
    regfile* file;
    std::condition_variable* cv;
public:
    regfile_ptr(regfile* file, std::condition_variable* cv)
        : file(file), cv(cv) {
    }

    regfile_ptr(const regfile_ptr&) = delete;

    regfile_ptr(std::nullptr_t) : file(nullptr), cv(nullptr) {
    }

    bool operator==(std::nullptr_t) const {
        return file == nullptr;
    }
    bool operator!=(std::nullptr_t) const {
        return file != nullptr;
    }
    operator bool() const {
        return file != nullptr;
    }
    ~regfile_ptr() {
        reset();
    }
    regfile* get() {
        return file;
    }
    void reset() {
        if (file) {
            file->inUse = false;
            cv->notify_one();
            file = nullptr;
        }
    }
};

inline void calc_reg_coords(
    int x, int z, int& regionX, int& regionZ, int& localX, int& localZ
) {
    regionX = floordiv(x, REGION_SIZE);
    regionZ = floordiv(z, REGION_SIZE);
    localX = x - (regionX * REGION_SIZE);
    localZ = z - (regionZ * REGION_SIZE);
}

struct RegionsLayer {
    /// @brief Layer index
    RegionLayerIndex layer;
    
    /// @brief Regions layer folder
    io::path folder;

    compression::Method compression = compression::Method::NONE;

    /// @brief In-memory regions data
    RegionsMap regions;

    /// @brief In-memory regions map mutex
    std::mutex mapMutex;

    /// @brief Open region files map
    std::unordered_map<glm::ivec2, std::unique_ptr<regfile>> openRegFiles;

    /// @brief Open region files map mutex
    std::mutex regFilesMutex;
    std::condition_variable regFilesCv;

    [[nodiscard]] regfile_ptr getRegFile(glm::ivec2 coord, bool create = true);
    [[nodiscard]] regfile_ptr useRegFile(glm::ivec2 coord);
    regfile_ptr createRegFile(glm::ivec2 coord);
    void closeRegFile(glm::ivec2 coord);

    WorldRegion* getRegion(int x, int z);
    WorldRegion* getOrCreateRegion(int x, int z);

    io::path getRegionFilePath(int x, int z) const;

    /// @brief Get chunk data. Read from file if not loaded yet.
    /// @param x chunk x coord
    /// @param z chunk z coord
    /// @param size [out] compressed chunk data length
    /// @param size [out] source chunk data length
    /// @return nullptr if no saved chunk data found
    [[nodiscard]] ubyte* getData(int x, int z, uint32_t& size, uint32_t& srcSize);

    /// @brief Write or rewrite region file
    /// @param x region X
    /// @param z region Z
    void writeRegion(int x, int y, WorldRegion* entry);

    /// @brief Write all unsaved regions to files
    void writeAll();

    /// @brief Read chunk data from region file
    /// @param x chunk x coord
    /// @param z chunk z coord
    /// @param size [out] compressed chunk data length
    /// @param srcSize [out] source chunk data length
    /// @param rfile region file
    /// @return nullptr if chunk is not present in region file
    [[nodiscard]] static std::unique_ptr<ubyte[]> readChunkData(
        int x, int z, uint32_t& size, uint32_t& srcSize, regfile* rfile
    );
};

class WorldRegions {
    /// @brief World directory
    io::path directory;

    RegionsLayer layers[REGION_LAYERS_COUNT] {};
public:
    bool generatorTestMode = false;
    bool doWriteLights = true;

    WorldRegions(const io::path& directory);
    WorldRegions(const WorldRegions&) = delete;
    ~WorldRegions();

    /// @brief Put all chunk data to regions
    void put(Chunk* chunk, std::vector<ubyte> entitiesData);

    /// @brief Store data in specified region
    /// @param x chunk.x
    /// @param z chunk.z
    /// @param layer regions layer
    /// @param data target data
    /// @param size data size
    void put(
        int x,
        int z,
        RegionLayerIndex layer,
        std::unique_ptr<ubyte[]> data,
        size_t size
    );

    /// @brief Get chunk voxels data
    /// @param x chunk.x
    /// @param z chunk.z
    /// @return voxels data buffer or nullptr
    std::unique_ptr<ubyte[]> getVoxels(int x, int z);

    /// @brief Get cached lights for chunk at x,z
    /// @return lights data or nullptr
    std::unique_ptr<light_t[]> getLights(int x, int z);
    
    ChunkInventoriesMap fetchInventories(int x, int z);

    BlocksMetadata getBlocksData(int x, int z);
    
    /// @brief Load saved entities data for chunk
    /// @param x chunk.x
    /// @param z chunk.z
    /// @return map with entities list as "data"
    dv::value fetchEntities(int x, int z);

    /// @brief Load, process and save processed region chunks data
    /// @param x region X
    /// @param z region Z
    /// @param layerid regions layer index
    /// @param func processing callback
    void processRegion(
        int x, int z, RegionLayerIndex layerid, const RegionProc& func);

    void processInventories(int x, int z, const InventoryProc& func);

    void processBlocksData(int x, int z, const BlockDataProc& func);

    /// @brief Get regions directory by layer index
    /// @param layerid layer index
    /// @return directory path
    const io::path& getRegionsFolder(RegionLayerIndex layerid) const;

    io::path getRegionFilePath(RegionLayerIndex layerid, int x, int z) const;

    /// @brief Write all region layers
    void writeAll();

    void deleteRegion(RegionLayerIndex layerid, int x, int z);

    /// @brief Extract X and Z from 'X_Z.bin' region file name.
    /// @param name source region file name
    /// @param x parsed X destination
    /// @param z parsed Z destination
    /// @return false if std::invalid_argument or std::out_of_range occurred
    static bool parseRegionFilename(const std::string& name, int& x, int& y);
};
