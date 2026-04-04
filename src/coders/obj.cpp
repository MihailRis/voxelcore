#include "obj.hpp"

#include "graphics/commons/Model.hpp"
#include "BasicParser.hpp"

using namespace model;

class ObjParser : BasicParser<char> {
    std::vector<glm::vec3> coords {{0, 0, 0}};
    std::vector<glm::vec2> uvs {{0, 0}};
    std::vector<glm::vec3> normals {{0, 1, 0}};

    // TODO: refactor
    void parseFace(Mesh& mesh) {
        std::vector<Vertex> vertices;
        while (hasNext()) {
            auto c = peekInLine();
            if (c == '\n') {
                break;
            } else if (hasNext()) {
                uint indices[3] {};
                uint i = 0;
                do {
                    char next = peekInLine();
                    if (is_digit(next)) {
                        indices[i] = parseSimpleInt(10);
                        if (hasNext() && peekInLine() == '/') {
                            pos++;
                        }
                    } else if (next == '/') {
                        pos++;
                    } else {
                        break;
                    }
                } while (hasNext() && peekInLine() != '\n' && ++i < 3);

                vertices.push_back(Vertex {
                    coords[indices[0]], uvs[indices[1]], normals[indices[2]]});
            }
        }
        if (hasNext() && peekInLine() != '\n') {
            skipLine();
        }
        if (vertices.size() >= 3) {
            for (size_t j = 0; j < vertices.size() - 2; j++) {
                mesh.vertices.push_back(vertices[0]);
                for (size_t i = 1; i < 3; i++) {
                    mesh.vertices.push_back(vertices[i + j]);
                }
            }
        }
    }
public:
    ObjParser(const std::string_view file, const std::string_view src)
        : BasicParser(file, src) {
    }

    std::unique_ptr<Model> parse() {
        // first iteration - collecting vertex data
        while (hasNext()) {
            if (peek() == '#') {
                skipLine();
                continue;
            }
            auto cmd = parseName();
            if (cmd == "v") {
                float x = parseNumber().asNumber();
                float y = parseNumber().asNumber();
                float z = parseNumber().asNumber();
                coords.emplace_back(x, y, z);
            } else if (cmd == "vt") {
                float u = parseNumber().asNumber();
                float v = parseNumber().asNumber();
                uvs.emplace_back(u, v);
            } else if (cmd == "vn") {
                float x = parseNumber().asNumber();
                float y = parseNumber().asNumber();
                float z = parseNumber().asNumber();
                normals.emplace_back(x, y, z);
            } else {
                skipLine();
            }
        }
        // second iteration - building meshes
        reset();

        auto model = std::make_unique<Model>();
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
            Mesh* mesh = &model->addMesh(texture);
            while (hasNext()) {
                if (peek() == '#') {
                    skipLine();
                    continue;
                }
                auto cmd = parseName();
                if (cmd == "usemtl") {
                    skipWhitespace();
                    texture = readUntilEOL();
                    mesh = &model->addMesh(texture);
                    break;
                } else if (cmd == "f") {
                    parseFace(*mesh);
                }
                skipLine();
            }
        } while (hasNext());
        model->clean();
        return model;
    }
};

std::unique_ptr<Model> obj::parse(
    const std::string_view file, const std::string_view src
) {
    return ObjParser(file, src).parse();
}
