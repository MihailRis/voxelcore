#include "audio/audio.hpp"
#include "delegates.hpp"
#include "engine/Engine.hpp"
#include "settings.hpp"
#include "hud.hpp"
#include "content/Content.hpp"
#include "graphics/core/Mesh.hpp"
#include "graphics/ui/elements/CheckBox.hpp"
#include "graphics/ui/elements/TextBox.hpp"
#include "graphics/ui/elements/TrackBar.hpp"
#include "graphics/ui/elements/InputBindBox.hpp"
#include "graphics/render/WorldRenderer.hpp"
#include "graphics/render/ParticlesRenderer.hpp"
#include "graphics/render/ChunksRenderer.hpp"
#include "graphics/render/DebugLinesRenderer.hpp"
#include "logic/scripting/scripting.hpp"
#include "network/Network.hpp"
#include "objects/Player.hpp"
#include "objects/Players.hpp"
#include "objects/Entities.hpp"
#include "objects/EntityDef.hpp"
#include "objects/Entity.hpp"
#include "physics/Hitbox.hpp"
#include "util/stringutil.hpp"
#include "voxels/Block.hpp"
#include "voxels/Chunk.hpp"
#include "voxels/Chunks.hpp"
#include "voxels/GlobalChunks.hpp"
#include "world/Level.hpp"
#include "world/World.hpp"

#include <string>
#include <memory>
#include <sstream>
#include <bitset>
#include <utility>

using namespace gui;

static std::shared_ptr<Label> create_label(GUI& gui, wstringsupplier supplier) {
    auto label = std::make_shared<Label>(gui, L"-");
    label->textSupplier(std::move(supplier));
    return label;
}

// TODO: move to xml
// TODO: move to xml finally
// TODO: move to xml finally
// TODO: move to xml finally
std::shared_ptr<UINode> create_debug_panel(
    Engine& engine, 
    Level& level, 
    Player& player,
    bool allowDebugCheats
) {
    auto& gui = engine.getGUI();
    auto panel = std::make_shared<Panel>(
        gui, glm::vec2(300, 200), glm::vec4(5.0f), 2.0f
    );
    panel->setId("hud.debug-panel");
    panel->setPos(glm::vec2(10, 10));

    static int fps = 0;
    static int fpsMin = fps;
    static int fpsMax = fps;
    static std::wstring fpsString = L"";

    static size_t lastTotalDownload = 0;
    static size_t lastTotalUpload = 0;
    static std::wstring netSpeedString = L"";

    panel->listenInterval(0.016f, [&engine]() {
        fps = 1.0f / engine.getTime().getDelta();
        fpsMin = std::min(fps, fpsMin);
        fpsMax = std::max(fps, fpsMax);
    });

    panel->listenInterval(0.5f, []() {
        fpsString = std::to_wstring(fpsMax)+L" / "+std::to_wstring(fpsMin);
        fpsMin = fps;
        fpsMax = fps;
    });

    panel->listenInterval(1.0f, [&engine]() {
        const auto& network = engine.getNetwork();
        size_t totalDownload = network.getTotalDownload();
        size_t totalUpload = network.getTotalUpload();
        netSpeedString =
            L"download: " + std::to_wstring(totalDownload - lastTotalDownload) +
            L" B/s upload: " + std::to_wstring(totalUpload - lastTotalUpload) +
            L" B/s";
        lastTotalDownload = totalDownload;
        lastTotalUpload = totalUpload;
    });

    panel->add(create_label(gui, []() { return L"fps: "+fpsString;}));
   
    panel->add(create_label(gui, []() {
        return L"meshes: " + std::to_wstring(MeshStats::meshesCount);
    }));
    panel->add(create_label(gui, []() {
        int drawCalls = MeshStats::drawCalls;
        MeshStats::drawCalls = 0;
        return L"draw-calls: " + std::to_wstring(drawCalls);
    }));
    panel->add(create_label(gui, []() {
        return L"speakers: " + std::to_wstring(audio::count_speakers())+
               L" streams: " + std::to_wstring(audio::count_streams());
    }));
    panel->add(create_label(gui, []() {
        return L"lua-stack: " + std::to_wstring(scripting::get_values_on_stack());
    }));
    panel->add(create_label(gui, []() { return netSpeedString; }));
    panel->add(create_label(gui, [&engine]() {
        auto& settings = engine.getSettings();
        bool culling = settings.graphics.frustumCulling.get();
        return L"frustum-culling: "+std::wstring(culling ? L"on" : L"off");
    }));
    panel->add(create_label(gui, [=]() {
        return L"particles: " +
               std::to_wstring(ParticlesRenderer::visibleParticles) +
               L" emitters: " +
               std::to_wstring(ParticlesRenderer::aliveEmitters);
    }));
    panel->add(create_label(gui, [&]() {
        return L"chunks: "+std::to_wstring(level.chunks->size())+
               L" visible: "+std::to_wstring(ChunksRenderer::visibleChunks);
    }));
    panel->add(create_label(gui, [&]() {
        return L"entities: "+std::to_wstring(level.entities->size())+L" next: "+
               std::to_wstring(level.entities->peekNextID());
    }));
    panel->add(create_label(gui, [&]() {
        return L"players: "+std::to_wstring(level.players->size())+L" local: "+
               std::to_wstring(player.getId());
    }));
    panel->add(create_label(gui, [&]() -> std::wstring {
        const auto& vox = player.selection.vox;
        std::wstringstream stream;
        stream << "r:" << vox.state.rotation << " s:"
                << std::bitset<3>(vox.state.segment) << " u:"
                << std::bitset<8>(vox.state.userbits);
        if (vox.id == BLOCK_VOID) {
            return L"block: -";
        } else {
            return L"block: "+std::to_wstring(vox.id)+
                   L" "+stream.str();
        }
    }));
    panel->add(create_label(gui, [&]() -> std::wstring {
        const auto& selection = player.selection;
        const auto& vox = selection.vox;
        if (vox.id == BLOCK_VOID) {
            return L"x: - y: - z: -";
        }
        return L"x: " + std::to_wstring(selection.actualPosition.x) +
               L" y: " + std::to_wstring(selection.actualPosition.y) +
               L" z: " + std::to_wstring(selection.actualPosition.z);
    }));
    panel->add(create_label(gui, [&]() {
        auto eid = player.getSelectedEntity();
        if (eid == ENTITY_NONE) {
            return std::wstring {L"entity: -"};
        } else if (auto entity = level.entities->get(eid)) {
            return L"entity: "+util::str2wstr_utf8(entity->getDef().name)+
                   L" uid: "+std::to_wstring(entity->getUID());
        } else {
            return std::wstring {L"entity: error (invalid UID)"};
        }
    }));
    panel->add(create_label(gui, [&](){
        auto indices = level.content.getIndices();
        if (auto def = indices->blocks.get(player.selection.vox.id)) {
            return L"name: " + util::str2wstr_utf8(def->name);
        } else {
            return std::wstring {L"name: void"};
        }
    }));
    panel->add(create_label(gui, [&](){
        return L"seed: "+std::to_wstring(level.getWorld()->getSeed());
    }));

    for (int ax = 0; ax < 3; ax++) {
        auto sub = std::make_shared<Container>(gui, glm::vec2(250, 27));

        std::wstring str = L"x: ";
        str[0] += ax;
        auto label = std::make_shared<Label>(gui, str);
        label->setMargin(glm::vec4(2, 3, 2, 3));
        label->setSize(glm::vec2(20, 27));
        sub->add(label);
        sub->setColor(glm::vec4(0.0f));

        // Coord input
        auto box = std::make_shared<TextBox>(gui, L"");
        auto boxRef = box.get();
        box->setTextSupplier([&player, ax]() {
            return util::to_wstring(player.getPosition()[ax], 2);
        });
        if (allowDebugCheats) {
            box->setTextConsumer([&player, ax](const std::wstring& text) {
                try {
                    glm::vec3 position = player.getPosition();
                    position[ax] = std::stoi(text);
                    player.teleport(position);
                } catch (std::exception& _){
                }
            });
        }
        box->setOnEditStart([&player, boxRef, ax]() {
            boxRef->setText(
                std::to_wstring(static_cast<int>(player.getPosition()[ax]))
            );
        });
        box->setSize(glm::vec2(230, 27));

        sub->add(box, glm::vec2(20, 0));
        panel->add(sub);
    }
    auto& worldInfo = level.getWorld()->getInfo();
    panel->add(create_label(gui, [&](){
        int hour, minute, second;
        timeutil::from_value(worldInfo.daytime, hour, minute, second);

        std::wstring timeString = 
                util::lfill(std::to_wstring(hour), 2, L'0') + L":" +
                util::lfill(std::to_wstring(minute), 2, L'0');
        return L"time: "+timeString;
    }));
    if (allowDebugCheats) {
        auto bar = std::make_shared<TrackBar>(gui, 0.0f, 1.0f, 1.0f, 0.005f, 8);
        bar->setSupplier([&]() {return worldInfo.daytime;});
        bar->setConsumer([&](double val) {worldInfo.daytime = val;});
        panel->add(bar);
    }
    if (allowDebugCheats) {
        auto bar = std::make_shared<TrackBar>(gui, 0.0f, 1.0f, 0.0f, 0.005f, 8);
        bar->setSupplier([&]() {return worldInfo.fog;});
        bar->setConsumer([&](double val) {worldInfo.fog = val;});
        panel->add(bar);
    }
    {
        auto checkbox = std::make_shared<FullCheckBox>(
            gui, L"Show Chunk Borders", glm::vec2(400, 24)
        );
        checkbox->setSupplier([=]() {
            return WorldRenderer::showChunkBorders;
        });
        checkbox->setConsumer([=](bool checked) {
            WorldRenderer::showChunkBorders = checked;
        });
        panel->add(checkbox);
    }
    {
        auto checkbox = std::make_shared<FullCheckBox>(
            gui, L"Show Hitboxes", glm::vec2(400, 24)
        );
        checkbox->setSupplier([=]() {
            return WorldRenderer::showEntitiesDebug;
        });
        checkbox->setConsumer([=](bool checked) {
            WorldRenderer::showEntitiesDebug = checked;
        });
        panel->add(checkbox);
    }
    {
        auto checkbox = std::make_shared<FullCheckBox>(
            gui, L"Show Paths", glm::vec2(400, 24)
        );
        checkbox->setSupplier([=]() {
            return DebugLinesRenderer::showPaths;
        });
        checkbox->setConsumer([=](bool checked) {
            DebugLinesRenderer::showPaths = checked;
        });
        panel->add(checkbox);
    }
    {
        auto checkbox = std::make_shared<FullCheckBox>(
            gui, L"Show Generator Minimap", glm::vec2(400, 24)
        );
        checkbox->setSupplier([=]() {
            return Hud::showGeneratorMinimap;
        });
        checkbox->setConsumer([=](bool checked) {
            Hud::showGeneratorMinimap = checked;
        });
        panel->add(checkbox);
    }
    panel->refresh();
    return panel;
}
