#include "compressed_chunks.hpp"

#include "coders/rle.hpp"
#include "coders/gzip.hpp"

#include "world/files/WorldFiles.hpp"
#include "content/Content.hpp"

inline constexpr int HAS_VOXELS = 0x1;
inline constexpr int HAS_METADATA = 0x2;

std::vector<ubyte> compressed_chunks::encode(
    const ubyte* data,
    const BlocksMetadata& metadata,
    util::Buffer<ubyte>& rleBuffer
) {
    size_t rleCompressedSize =
        extrle::encode16(data, CHUNK_DATA_LEN, rleBuffer.data());

    const auto gzipCompressedData = gzip::compress(
        rleBuffer.data(), rleCompressedSize
    );
    auto metadataBytes = metadata.serialize();

    ByteBuilder builder(2 + 8 + gzipCompressedData.size() + metadataBytes.size());
    builder.put(HAS_VOXELS | HAS_METADATA); // flags
    builder.put(0); // reserved
    builder.putInt32(gzipCompressedData.size());
    builder.put(gzipCompressedData.data(), gzipCompressedData.size());
    builder.putInt32(metadataBytes.size());
    builder.put(metadataBytes.data(), metadataBytes.size());
    return builder.build();
}

std::vector<ubyte> compressed_chunks::encode(const Chunk& chunk) {
    auto data = chunk.encode();

    /// world.get_chunk_data is only available in the main Lua state
    static util::Buffer<ubyte> rleBuffer(CHUNK_DATA_LEN * 2);
    return encode(data.get(), chunk.blocksMetadata, rleBuffer);
}

static void read_voxel_data(ByteReader& reader, util::Buffer<ubyte>& dst) {
    size_t gzipCompressedSize = reader.getInt32();
        
    auto rleData = gzip::decompress(reader.pointer(), gzipCompressedSize);
    reader.skip(gzipCompressedSize);

    extrle::decode16(rleData.data(), rleData.size(), dst.data(), dst.size());
}

void compressed_chunks::decode(
    Chunk& chunk, const ubyte* src, size_t size, const ContentIndices& indices
) {
    ByteReader reader(src, size);

    ubyte flags = reader.get();
    reader.skip(1); // reserved byte

    if (flags & HAS_VOXELS) {
        /// world.get_chunk_data is only available in the main Lua state
        static util::Buffer<ubyte> voxelData (CHUNK_DATA_LEN);
        read_voxel_data(reader, voxelData);
        // TODO: move somewhere in Chunk
        auto src = reinterpret_cast<const uint16_t*>(voxelData.data());
        for (size_t i = 0; i < CHUNK_VOL; i++) {
            blockid_t id = dataio::le2h(src[i]);;
            if (indices.blocks.get(id) == nullptr) {
                throw std::runtime_error(
                    "block data corruption (chunk: " + std::to_string(chunk.x) +
                    ", " + std::to_string(chunk.z) + ") at " +
                    std::to_string(i) + " id: " + std::to_string(id)
                );
            }
        }
        chunk.decode(voxelData.data());
        chunk.updateHeights();
    }
    if (flags & HAS_METADATA) {
        size_t metadataSize = reader.getInt32();
        chunk.blocksMetadata.deserialize(reader.pointer(), metadataSize);
        reader.skip(metadataSize);
    }
    chunk.setModifiedAndUnsaved();
}

void compressed_chunks::save(
    int x, int z, std::vector<ubyte> bytes, WorldRegions& regions
) {
    ByteReader reader(bytes.data(), bytes.size());

    ubyte flags = reader.get();
    reader.skip(1); // reserved byte
    if (flags & HAS_VOXELS) {
        util::Buffer<ubyte> voxelData (CHUNK_DATA_LEN);
        read_voxel_data(reader, voxelData);
        regions.put(
            x, z, REGION_LAYER_VOXELS, voxelData.release(), CHUNK_DATA_LEN
        );
    }
    if (flags & HAS_METADATA) {
        size_t metadataSize = reader.getInt32();
        regions.put(
            x,
            z,
            REGION_LAYER_BLOCKS_DATA,
            util::Buffer<ubyte>(reader.pointer(), metadataSize).release(),
            metadataSize
        );
        reader.skip(metadataSize);
    }
}
