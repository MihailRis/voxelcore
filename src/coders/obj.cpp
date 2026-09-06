#include "obj.hpp"

#include "graphics/commons/Model.hpp"
#include "BasicParser.hpp"

using namespace model;

class ObjParser : BasicParser<char> {
    std::vector<glm::vec3> coords {{0, 0, 0}};
    std::vector<glm::vec2> uvs {{0, 0}};
    std::vector<glm::vec3> normals {{0, 1, 0}};

    Vertex parseFaceVertex() {
        auto parseIndex = [&]() -> uint {
            if (!hasNext() || !is_digit(peekInLine())) {
                return 0;
            }
            return parseSimpleInt(10);
        };

        uint coord = parseIndex();
        uint uv = 0;
        uint normal = 0;

        if (hasNext() && peekInLine() == '/') {
            pos++;

            uv = parseIndex();

            if (hasNext() && peekInLine() == '/') {
                pos++;
                normal = parseIndex();
            }
        }

        return Vertex {
            coords[coord], 
            uvs[uv], 
            normals[normal]
        };
    }

    void triangulate(const std::vector<Vertex>& vertices, std::vector<Vertex>& output) {
        if (vertices.size() >= 3) {
            for (size_t j = 0; j < vertices.size() - 2; j++) {
                output.push_back(vertices[0]);
                for (size_t i = 1; i < 3; i++) {
                    output.push_back(vertices[i + j]);
                }
            }
        }
    }

    void parseFace(Mesh& mesh) {
        std::vector<Vertex> vertices;
        while (hasNext() && peekInLine() != '\n') {
            vertices.push_back(parseFaceVertex());
        }
        triangulate(vertices, mesh.vertices);
    }

    glm::vec3 parseVec3() {
        float x = parseNumber().asNumber();
        float y = parseNumber().asNumber();
        float z = parseNumber().asNumber();
        return glm::vec3 {x, y, z};
    }

    glm::vec2 parseVec2() {
        float x = parseNumber().asNumber();
        float y = parseNumber().asNumber();
        return glm::vec2 {x, y};
    }

    void collectVertexData() {
        while (hasNext()) {
            if (peek() == '#') {
                skipLine();
                continue;
            }
            auto cmd = parseName();
            if (cmd == "v") {
                coords.emplace_back(parseVec3());
            } else if (cmd == "vt") {
                uvs.emplace_back(parseVec2());
            } else if (cmd == "vn") {
                normals.emplace_back(parseVec3());
            } else {
                skipLine();
            }
        }
    }

    void buildMeshes(Model& model) {
        std::string texture;
        while (hasNext()) {
            if (peek() != '#' && parseName() == "usemtl") {
                skipWhitespace();
                texture = readUntilEOL();
                break;
            }
            skipLine();
        }
        do {
            Mesh* mesh = &model.addMesh(texture);
            while (hasNext()) {
                if (peek() == '#') {
                    skipLine();
                    continue;
                }
                auto cmd = parseName();
                if (cmd == "usemtl") {
                    skipWhitespace();
                    texture = readUntilEOL();
                    mesh = &model.addMesh(texture);
                    break;
                } else if (cmd == "f") {
                    parseFace(*mesh);
                }
                skipLine();
            }
        } while (hasNext());
    }
public:
    ObjParser(const std::string_view file, const std::string_view src)
        : BasicParser(file, src) {
    }

    std::unique_ptr<Model> parse() {
        // first iteration - collecting vertex data
        collectVertexData();
        // second iteration - building meshes
        reset();
        auto model = std::make_unique<Model>();
        buildMeshes(*model);
        model->clean();
        return model;
    }
};

std::unique_ptr<Model> obj::parse(
    const std::string_view file, const std::string_view src
) {
    return ObjParser(file, src).parse();
}
