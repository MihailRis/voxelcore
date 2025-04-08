#pragma once

#include <functional>
#include <memory>
#include <string>

#include "typedefs.hpp"
#include "io/fwd.hpp"

class Hud;
class WorldRenderer;
class PostProcessing;

namespace gui {
    class UINode;
    using PageLoaderFunc = std::function<std::shared_ptr<UINode>(const std::string&)>;
}

namespace scripting {
    extern Hud *hud;
    extern WorldRenderer* renderer;
    extern PostProcessing* post_processing;

    void on_frontend_init(
        Hud* hud, WorldRenderer* renderer, PostProcessing* postProcessing
    );
    void on_frontend_render();
    void on_frontend_close();

    /// @brief Load package-specific hud script
    /// @param env environment id
    /// @param packid content-pack id
    /// @param file script file path
    /// @param fileName script file path using the engine format
    void load_hud_script(
        const scriptenv& env,
        const std::string& packid,
        const io::path& file,
        const std::string& fileName
    );

    gui::PageLoaderFunc create_page_loader(); 
}
