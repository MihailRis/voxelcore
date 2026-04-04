#include "vec3.hpp"

#include <stdexcept>

#include "byte_utils.hpp"
#include "util/data_io.hpp"
#include "util/stringutil.hpp"
#include "graphics/commons/Model.hpp"

inline constexpr int VERSION = 1;

inline constexpr int FLAG_ZLIB = 0x1;
inline constexpr int FLAG_16BIT_INDICES = 0x2;

using namespace vec3;

vec3::Model::~Model() = default;

enum AttributeType {
    POSITION = 0,
    UV,
    NORMAL,
    COLOR
};

enum MaterialFlag {
    SHADELESS = 1
};

struct VertexAttribute {
    AttributeType type;
    int flags;
    util::Buffer<float> data;

    VertexAttribute() = default;

    VertexAttribute(VertexAttribute&&) = default;

    VertexAttribute& operator=(VertexAttribute&& o) {
        type = o.type;
        flags = o.flags;
        data = std::move(o.data);
        return *this;
    }
};

static VertexAttribute load_attribute(ByteReader& reader) {
    auto type = static_cast<AttributeType>(reader.get());
    int flags = reader.get();
    assert(type >= POSITION && flags <= COLOR);
    if (flags != 0) {
        throw std::runtime_error("attribute compression is not supported yet");
    }
    int size = reader.getInt32();

    util::Buffer<float> data(size / sizeof(float));
    reader.get(reinterpret_cast<char*>(data.data()), size);
    if (dataio::is_big_endian()) {
        for (int i = 0; i < data.size(); i++) {
            data[i] = dataio::swap(data[i]);
        }
    }
    return VertexAttribute {type, flags, std::move(data)};
}

static model::Mesh build_mesh(
    const std::vector<VertexAttribute>& attrs, 
    const util::Buffer<uint16_t>& indices,
    const Material& material
) {
    const glm::vec3* coords = nullptr;
    const glm::vec2* uvs = nullptr;
    const glm::vec3* normals = nullptr;

    int coordsIndex = 0, uvsIndex = 0, normalsIndex = 0;

    for (int i = 0; i < attrs.size(); i++) {
        const auto& attr = attrs[i];
        switch (attr.type) {
            case POSITION:
                coords = reinterpret_cast<const glm::vec3*>(attr.data.data());
                coordsIndex = i;
                break;
            case UV:
                uvs = reinterpret_cast<const glm::vec2*>(attr.data.data());
                uvsIndex = i;
                break;
            case NORMAL:
                normals = reinterpret_cast<const glm::vec3*>(attr.data.data());
                normalsIndex = i;
                break;
            case COLOR: // unused
                break;
        }
    }
    std::vector<model::Vertex> vertices;
    int attrsCount = attrs.size();
    int verticesCount = indices.size() / attrsCount;
    for (int i = 0; i < verticesCount; i++) {
        model::Vertex vertex {};
        if (coords) {
            vertex.coord = coords[indices[i * attrsCount + coordsIndex]];
        }
        if (uvs) {
            vertex.uv = uvs[indices[i * attrsCount + uvsIndex]];
        }
        if (normals) {
            vertex.normal = normals[indices[i * attrsCount + normalsIndex]];
        } else if (coords) {
            // Flat normal calculation
            int idx = (i / 3) * 3;
            auto a = coords[indices[idx * attrsCount + coordsIndex]];
            auto b = coords[indices[(idx + 1) * attrsCount + coordsIndex]];
            auto c = coords[indices[(idx + 2) * attrsCount + coordsIndex]];
            vertex.normal = glm::normalize(glm::cross(b - a, c - a));
        }
        vertices.push_back(std::move(vertex));
    }
    return model::Mesh {
        material.name,
        std::move(vertices),
        (material.flags & MaterialFlag::SHADELESS) == 0
    };
}

static model::Mesh load_mesh(
    ByteReader& reader, const std::vector<Material>& materials
) {
    int triangleCount = reader.getInt32();
    int materialId = reader.getInt16();
    int flags = reader.getInt16();
    int attributeCount = reader.getInt16();
    if (flags == FLAG_ZLIB) {
        throw std::runtime_error("compression is not supported yet");
    }
    std::vector<VertexAttribute> attributes;
    for (int i = 0; i < attributeCount; i++) {
        attributes.push_back(load_attribute(reader));
    }

    util::Buffer<uint16_t> indices(triangleCount * 3 * attributeCount);
    if ((flags & FLAG_16BIT_INDICES) == 0){
        util::Buffer<uint8_t> smallIndices(indices.size());
        reader.get(
            reinterpret_cast<char*>(smallIndices.data()),
            indices.size() * sizeof(uint8_t)
        );
        for (int i = 0; i < indices.size(); i++) {
            indices[i] = smallIndices[i];
        }
    } else {
        reader.get(
            reinterpret_cast<char*>(indices.data()),
            indices.size() * sizeof(uint16_t)
        );
    }
    if (dataio::is_big_endian()) {
        for (int i = 0; i < indices.size(); i++) {
            indices[i] = dataio::swap(indices[i]);
        }
    }
    return build_mesh(
        attributes,
        indices,
        materials.at(materialId)
    );
}

static Model load_model(
    ByteReader& reader, const std::vector<Material>& materials
) {
    int nameLength = reader.getInt16();
    assert(nameLength >= 0);
    float x = reader.getFloat32();
    float y = reader.getFloat32();
    float z = reader.getFloat32();
    int meshCount = reader.getInt32();
    assert(meshCount >= 0);
    
    std::vector<model::Mesh> meshes;
    for (int i = 0; i < meshCount; i++) {
        meshes.push_back(load_mesh(reader, materials));
    }
    util::Buffer<char> chars(nameLength);
    reader.get(chars.data(), nameLength);
    std::string name(chars.data(), nameLength);
    
    glm::vec3 offset {x, y, z};
    for (auto& mesh : meshes) {
        for (auto& vertex : mesh.vertices) {
            vertex.coord -= offset;
        }
    }
    return Model {std::move(name), model::Model {std::move(meshes)}, {x, y, z}};
}

static Material load_material(ByteReader& reader) {
    int flags = reader.getInt16();
    int nameLength = reader.getInt16();
    assert(nameLength >= 0);
    util::Buffer<char> chars(nameLength);
    reader.get(chars.data(), nameLength);
    std::string name(chars.data(), nameLength);
    return Material {flags, std::move(name)};
}

File vec3::load(
    const std::string_view file, const util::Buffer<ubyte>& src
) {
    ByteReader reader(src.data(), src.size());
    
    // Header
    reader.checkMagic("\0\0VEC3\0\0", 8);
    int version = reader.getInt16();
    [[maybe_unused]] int reserved = reader.getInt16();
    if (version > VERSION) {
        throw std::runtime_error("unsupported VEC3 version");
    }
    assert(reserved == 0);
    
    // Body
    int materialCount = reader.getInt16();
    int modelCount = reader.getInt16();
    assert(materialCount >= 0);
    assert(modelCount >= 0);

    std::vector<Material> materials;
    for (int i = 0; i < materialCount; i++) {
        materials.push_back(load_material(reader));
    }

    std::unordered_map<std::string, Model> models;
    for (int i = 0; i < modelCount; i++) {
        Model model = load_model(reader, materials);
        models[model.name] = std::move(model);
    }
    return File {std::move(models), std::move(materials)};
}
