#pragma once

#include "typedefs.hpp"
#include "commons.hpp"

#include <memory>
#include <vector>

template<typename VertexStructure> class Mesh;
struct PostProcessingVertex;
class Shader;

class TranslucentLayers {
public:
    TranslucentLayers(size_t count, uint width, uint height);
    ~TranslucentLayers();

    void resize(uint width, uint height);

    void prepare(uint sourceFbo);

    Bindable& getLayer(size_t index);

    void compose(Shader& shader);

private:
    class Layer : public Bindable {
    public:
        uint fbo = 0;
        uint color = 0;
        uint depth = 0;

        Layer();
        ~Layer() override;

        void bind() const override;
        void unbind() const override;

        void allocate(uint width, uint height);
    };

    uint width;
    uint height;
    std::vector<std::unique_ptr<Layer>> layers;
    std::unique_ptr<Mesh<PostProcessingVertex>> quadMesh;
};
