#define VC_ENABLE_REFLECTION
#include "libgui.hpp"
#include "assets/Assets.hpp"
#include "engine/Engine.hpp"
#include "frontend/locale.hpp"
#include "graphics/ui/GUI.hpp"
#include "graphics/ui/elements/Button.hpp"
#include "graphics/ui/elements/Canvas.hpp"
#include "graphics/ui/elements/CheckBox.hpp"
#include "graphics/ui/elements/Image.hpp"
#include "graphics/ui/elements/SelectBox.hpp"
#include "graphics/ui/elements/InventoryView.hpp"
#include "graphics/ui/elements/Menu.hpp"
#include "graphics/ui/elements/Panel.hpp"
#include "graphics/ui/elements/TextBox.hpp"
#include "graphics/ui/elements/TrackBar.hpp"
#include "graphics/ui/elements/InlineFrame.hpp"
#include "graphics/ui/elements/ModelViewer.hpp"
#include "graphics/ui/gui_util.hpp"
#include "graphics/ui/markdown.hpp"
#include "graphics/core/Font.hpp"
#include "items/Inventories.hpp"
#include "util/stringutil.hpp"
#include "world/Level.hpp"

using namespace gui;
using namespace scripting;

static DocumentNode get_document_node_impl(
    lua::State*, const std::string& name, const std::string& nodeName
) {
    auto doc = engine->getAssets()->get<UiDocument>(name);
    if (doc == nullptr) {
        throw std::runtime_error("document '" + name + "' not found");
    }
    auto node = doc->get(nodeName);
    if (node == nullptr) {
        throw std::runtime_error(
            "document '" + name + "' has no element with id '" + nodeName + "'"
        );
    }
    return {doc, node};
}

DocumentNode get_document_node(lua::State* L, int idx) {
    lua::getfield(L, "docname", idx);
    lua::getfield(L, "name", idx);
    auto docname = lua::require_string(L, -2);
    auto name = lua::require_string(L, -1);
    auto node = get_document_node_impl(L, docname, name);
    lua::pop(L, 2);
    return node;
}

static int l_menu_back(lua::State* L) {
    auto node = get_document_node(L);
    if (auto menu = dynamic_cast<Menu*>(node.node.get())) {
        menu->back();
    }
    return 0;
}

static int l_menu_reset(lua::State* L) {
    auto node = get_document_node(L);
    if (auto menu = dynamic_cast<Menu*>(node.node.get())) {
        menu->reset();
    }
    return 0;
}

static int l_textbox_paste(lua::State* L) {
    auto node = get_document_node(L);
    if (auto box = dynamic_cast<TextBox*>(node.node.get())) {
        auto text = lua::require_string(L, 2);
        box->paste(util::str2wstr_utf8(text));
    }
    return 0;
}

static int l_container_add(lua::State* L) {
    auto docnode = get_document_node(L);
    if (docnode.document == nullptr) {
        throw std::runtime_error("target document not found");
    }
    auto node = dynamic_cast<Container*>(docnode.node.get());
    if (node == nullptr) {
        throw std::runtime_error("target container not found");
    }
    auto xmlsrc = lua::require_string(L, 2);
    try {
        auto env = docnode.document->getEnvironment();
        if (lua::istable(L, 3)) {
            env = create_environment(std::move(env));
            lua::pushenv(L, *env);
            lua::pushvalue(L, 3);
            lua::setfield(L, "DATA");
            lua::pop(L);
        }
        auto subnode = guiutil::create(
            engine->getGUI(), xmlsrc, std::move(env)
        );
        UINode::getIndices(subnode, docnode.document->getMapWriteable());
        node->add(std::move(subnode));
    } catch (const std::exception& err) {
        throw std::runtime_error("container:add(...): " + std::string(err.what()));
    }
    return 0;
}

static int l_node_destruct(lua::State* L) {
    auto docnode = get_document_node(L);
    auto node = docnode.node;
    engine->getGUI().postRunnable([node]() {
        auto parent = node->getParent();
        if (auto container = dynamic_cast<Container*>(parent)) {
            container->remove(node.get());
        }
    });
    return 0;
}

static int l_container_refresh(lua::State* L) {
    auto docnode = get_document_node(L);
    auto node = dynamic_cast<Container*>(docnode.node.get());
    if (node == nullptr) {
        return 0;
    }
    node->refresh();
    return 0;
}

static int l_node_reposition(lua::State* L) {
    auto docnode = get_document_node(L);
    docnode.node->reposition();
    return 0;
}

static int l_container_clear(lua::State* L) {
    auto node = get_document_node(L, 1);
    if (auto container = std::dynamic_pointer_cast<Container>(node.node)) {
        container->clear();
    }
    return 0;
}

static int l_container_set_interval(lua::State* L) {
    auto node = get_document_node(L, 1);
    auto interval = lua::tointeger(L, 2) / 1000.0f;
    if (auto container = std::dynamic_pointer_cast<Container>(node.node)) {
        lua::pushvalue(L, 3);
        auto runnable = lua::create_runnable(L);
        container->listenInterval(interval, runnable);
    }
    return 0;
}

static int l_move_into(lua::State* L) {
    auto node = get_document_node(L, 1);
    auto dest = get_document_node(L, 2);
    UINode::moveInto(
        node.node, std::dynamic_pointer_cast<Container>(dest.node)
    );
    return 0;
}

static int l_get_line_at(lua::State* L) {
    auto node = get_document_node(L, 1);
    auto position = lua::tointeger(L, 2);
    if (auto box = dynamic_cast<TextBox*>(node.node.get())) {
        return lua::pushinteger(L, box->getLineAt(position));
    }
    return 0;
}

static int l_get_line_pos(lua::State* L) {
    auto node = get_document_node(L, 1);
    auto line = lua::tointeger(L, 2);
    if (auto box = dynamic_cast<TextBox*>(node.node.get())) {
        return lua::pushinteger(L, box->getLinePos(line));
    }
    return 0;
}

static int p_get_inventory(UINode* node, lua::State* L) {
    if (auto inventory = dynamic_cast<InventoryView*>(node)) {
        auto inv = inventory->getInventory();
        return lua::pushinteger(L, inv ? inv->getId() : 0);
    }
    return 0;
}

static int p_get_reset(UINode* node, lua::State* L) {
    if (dynamic_cast<Menu*>(node)) {
        return lua::pushcfunction(L, l_menu_reset);
    }
    return 0;
}

static int p_get_back(UINode* node, lua::State* L) {
    if (dynamic_cast<Menu*>(node)) {
        return lua::pushcfunction(L, l_menu_back);
    }
    return 0;
}

static int p_get_paste(UINode* node, lua::State* L) {
    if (dynamic_cast<TextBox*>(node)) {
        return lua::pushcfunction(L, l_textbox_paste);
    }
    return 0;
}

static int p_get_page(UINode* node, lua::State* L) {
    if (auto menu = dynamic_cast<Menu*>(node)) {
        return lua::pushstring(L, menu->getCurrent().name);
    }
    return 0;
}

static int p_is_checked(UINode* node, lua::State* L) {
    if (auto box = dynamic_cast<CheckBox*>(node)) {
        return lua::pushboolean(L, box->isChecked());
    } else if (auto box = dynamic_cast<FullCheckBox*>(node)) {
        return lua::pushboolean(L, box->isChecked());
    }
    return 0;
}

static int p_get_value(UINode* node, lua::State* L) {
    if (auto bar = dynamic_cast<TrackBar*>(node)) {
        return lua::pushnumber(L, bar->getValue());
    } else if (auto box = dynamic_cast<SelectBox*>(node)) {
        return lua::pushstring(L, box->getSelected().value);
    }
    return 0;
}

static int p_get_min(UINode* node, lua::State* L) {
    if (auto bar = dynamic_cast<TrackBar*>(node)) {
        return lua::pushnumber(L, bar->getMin());
    }
    return 0;
}

static int p_get_max(UINode* node, lua::State* L) {
    if (auto bar = dynamic_cast<TrackBar*>(node)) {
        return lua::pushnumber(L, bar->getMax());
    }
    return 0;
}

static int p_get_step(UINode* node, lua::State* L) {
    if (auto bar = dynamic_cast<TrackBar*>(node)) {
        return lua::pushnumber(L, bar->getStep());
    }
    return 0;
}

static int p_get_track_width(UINode* node, lua::State* L) {
    if (auto bar = dynamic_cast<TrackBar*>(node)) {
        return lua::pushnumber(L, bar->getTrackWidth());
    }
    return 0;
}

static int p_get_track_color(UINode* node, lua::State* L) {
    if (auto bar = dynamic_cast<TrackBar*>(node)) {
        return lua::pushcolor(L, bar->getTrackColor());
    }
    return 0;
}

static int p_get_text_color(UINode* node, lua::State* L) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        return lua::pushcolor(L, box->getTextColor());
    }
    return 0;
}

static int p_is_valid(UINode* node, lua::State* L) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        return lua::pushboolean(L, box->validate());
    }
    return 0;
}

static int p_get_caret(UINode* node, lua::State* L) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        return lua::pushinteger(L, static_cast<integer_t>(box->getCaret()));
    }
    return 0;
}

static int p_get_placeholder(UINode* node, lua::State* L) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        return lua::pushwstring(L, box->getPlaceholder());
    }
    return 0;
}

static int p_get_hint(UINode* node, lua::State* L) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        return lua::pushwstring(L, box->getHint());
    }
    return 0;
}

static int p_get_text(UINode* node, lua::State* L) {
    if (auto button = dynamic_cast<Button*>(node)) {
        return lua::pushwstring(L, button->getText());
    } else if (auto label = dynamic_cast<Label*>(node)) {
        return lua::pushwstring(L, label->getText());
    } else if (auto box = dynamic_cast<TextBox*>(node)) {
        return lua::pushwstring(L, box->getText());
    }
    return 0;
}

static int p_get_editable(UINode* node, lua::State* L) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        return lua::pushboolean(L, box->isEditable());
    }
    return 0;
}

static int p_get_edited(UINode* node, lua::State* L) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        return lua::pushboolean(L, box->isEdited());
    }
    return 0;
}

static int p_get_line_numbers(UINode* node, lua::State* L) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        return lua::pushboolean(L, box->isShowLineNumbers());
    }
    return 0;
}

static int p_get_syntax(UINode* node, lua::State* L) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        return lua::pushstring(L, box->getSyntax());
    }
    return 0;
}

static int p_get_markup(UINode* node, lua::State* L) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        return lua::pushstring(L, box->getMarkup());
    } else if (auto label = dynamic_cast<Label*>(node)) {
        return lua::pushstring(L, label->getMarkup());
    }
    return 0;
}

static int p_get_src(UINode* node, lua::State* L) {
    if (auto image = dynamic_cast<Image*>(node)) {
        return lua::pushstring(L, image->getTexture());
    } else if (auto iframe = dynamic_cast<InlineFrame*>(node)) {
        return lua::pushstring(L, iframe->getSrc());
    } else if (auto modelviewer = dynamic_cast<ModelViewer*>(node)) {
        return lua::pushstring(L, modelviewer->getModel());
    }
    return 0;
}

static int p_get_region(UINode* node, lua::State* L) {
    if (auto image = dynamic_cast<Image*>(node)) {
        const auto& region = image->getRegion();
        return lua::pushvec4(L, {region.u1, region.v1, region.u2, region.v2});
    }
    return 0;
}

static int p_get_data(UINode* node, lua::State* L) {
    if (auto canvas = dynamic_cast<Canvas*>(node)) {
        return lua::newuserdata<lua::LuaCanvas>(L, canvas->texture(), canvas->data());
    }
    return 0;
}

static const std::string& request_node_id(const DocumentNode& docnode) {
    std::string id = docnode.node->getId();
    if (id.empty()) {
        id = "#" + std::to_string(
            reinterpret_cast<std::ptrdiff_t>(docnode.node.get()));
    }
    docnode.node->setId(std::move(id));
    UINode::getIndices(
        docnode.node, docnode.document->getMapWriteable()
    );
    return docnode.node->getId();
}

/// @brief Push UI-document node object to stack
/// using lua argument at 1 as document name
/// @param id UI-node id
static int push_document_node(lua::State* L, const std::string& id) {
    lua::requireglobal(L, "__vc_get_document_node");
    lua::pushvalue(L, 1);
    lua::pushstring(L, id);
    return lua::call(L, 2, 1);
}

static int p_get_parent(UINode* node, lua::State* L) {
    auto parent = node->getParent();
    if (!parent) {
        return 0;
    }
    auto docname = lua::require_string(L, 1);
    auto element = lua::require_string(L, 2);
    auto docnode = get_document_node_impl(L, docname, element);

    const auto& id = request_node_id(docnode);

    return push_document_node(L, id);
}

static int p_get_add(UINode* node, lua::State* L) {
    if (dynamic_cast<Container*>(node)) {
        return lua::pushcfunction(L, lua::wrap<l_container_add>);
    }
    return 0;
}

static int p_get_destruct(UINode*, lua::State* L) {
    return lua::pushcfunction(L, lua::wrap<l_node_destruct>);
}

static int p_refresh(UINode* node, lua::State* L) {
    if (dynamic_cast<Container*>(node)) {
        return lua::pushcfunction(L, lua::wrap<l_container_refresh>);
    }
    return 0;
}

static int p_get_reposition(UINode*, lua::State* L) {
    return lua::pushcfunction(L, lua::wrap<l_node_reposition>);
}

static int p_get_clear(UINode* node, lua::State* L) {
    if (dynamic_cast<Container*>(node)) {
        return lua::pushcfunction(L, lua::wrap<l_container_clear>);
    }
    return 0;
}

static int p_set_interval(UINode* node, lua::State* L) {
    if (dynamic_cast<Container*>(node)) {
        return lua::pushcfunction(L, lua::wrap<l_container_set_interval>);
    }
    return 0;
}

static int p_get_content_offset(UINode* node, lua::State* L) {
    return lua::pushvec(L, node->getContentOffset());
}
static int p_get_id(UINode* node, lua::State* L) {
    return lua::pushstring(L, node->getId());
}
static int p_get_color(UINode* node, lua::State* L) {
    return lua::pushcolor(L, node->getColor());
}
static int p_get_hover_color(UINode* node, lua::State* L) {
    return lua::pushcolor(L, node->getHoverColor());
}
static int p_get_pressed_color(UINode* node, lua::State* L) {
    return lua::pushcolor(L, node->getPressedColor());
}
static int p_get_tooltip(UINode* node, lua::State* L) {
    return lua::pushwstring(L, node->getTooltip());
}
static int p_get_tooltip_delay(UINode* node, lua::State* L) {
    return lua::pushnumber(L, node->getTooltipDelay());
}
static int p_get_pos(UINode* node, lua::State* L) {
    return lua::pushvec2(L, node->getPos());
}
static int p_get_wpos(UINode* node, lua::State* L) {
    return lua::pushvec2(L, node->calcPos());
}
static int p_get_size(UINode* node, lua::State* L) {
    return lua::pushvec2(L, node->getSize());
}
static int p_is_interactive(UINode* node, lua::State* L) {
    return lua::pushboolean(L, node->isInteractive());
}
static int p_is_visible(UINode* node, lua::State* L) {
    return lua::pushboolean(L, node->isVisible());
}
static int p_is_enabled(UINode* node, lua::State* L) {
    return lua::pushboolean(L, node->isEnabled());
}
static int p_move_into(UINode*, lua::State* L) {
    return lua::pushcfunction(L, l_move_into);
}
static int p_get_focused(UINode* node, lua::State* L) {
    return lua::pushboolean(L, node->isFocused());
}
static int p_get_line_at(UINode*, lua::State* L) {
    return lua::pushcfunction(L, l_get_line_at);
}
static int p_get_line_pos(UINode*, lua::State* L) {
    return lua::pushcfunction(L, l_get_line_pos);
}

static int p_get_cursor(UINode* node, lua::State* L) {
    return lua::pushlstring(L, CursorShapeMeta.getName(node->getCursor()));
}

static int p_get_scroll(UINode* node, lua::State* L) {
    if (auto container = dynamic_cast<Container*>(node)) {
        return lua::pushnumber(L, container->getContentOffset().y);
    }
    return 0;
}

static int p_get_options(UINode* node, lua::State* L) {
    if (auto selectbox = dynamic_cast<SelectBox*>(node)) {
        const auto& options = selectbox->getOptions();
        size_t size = options.size();
        lua::createtable(L, size, 0);
        for (size_t i = 0; i < size; i++) {
            const auto& option = options[i];
            lua::createtable(L, 0, 2);

            lua::pushstring(L, option.value);
            lua::setfield(L, "value");

            lua::pushwstring(L, option.text);
            lua::setfield(L, "text");

            lua::rawseti(L, i + 1);
        }
        return 1;
    }
    return 0;
}

static int l_gui_getattr(lua::State* L) {
    if (!lua::isstring(L, 1)) {
        throw std::runtime_error("document name is not a string");
    }
    if (!lua::isstring(L, 2)) {
        throw std::runtime_error("element id is not a string");
    }
    auto docname = lua::require_string(L, 1);
    auto element = lua::require_string(L, 2);
    if (lua::isnumber(L, 3)) {
        auto docnode = get_document_node_impl(L, docname, element);
        auto container = dynamic_cast<Container*>(docnode.node.get());
        if (container == nullptr) {
            return 0;
        }
        size_t index = lua::tointeger(L, 3) - 1;
        const auto& nodes = container->getNodes();
        if (index >= nodes.size()) {
            return 0;
        }
        const auto& node = nodes.at(index);
        const auto& id = request_node_id(DocumentNode {docnode.document, node});
        return push_document_node(L, id);
    }
    if (!lua::isstring(L, 3)) {
        throw std::runtime_error("attribute name is not a string");
    }
    auto attr = lua::require_string(L, 3);

    static const std::unordered_map<
        std::string_view,
        std::function<int(UINode*, lua::State*)>>
        getters {
            {"id", p_get_id},
            {"color", p_get_color},
            {"hoverColor", p_get_hover_color},
            {"pressedColor", p_get_pressed_color},
            {"contentOffset", p_get_content_offset},
            {"tooltip", p_get_tooltip},
            {"tooltipDelay", p_get_tooltip_delay},
            {"pos", p_get_pos},
            {"wpos", p_get_wpos},
            {"size", p_get_size},
            {"interactive", p_is_interactive},
            {"visible", p_is_visible},
            {"enabled", p_is_enabled},
            {"move_into", p_move_into},  // deprecated
            {"moveInto", p_move_into},
            {"add", p_get_add},
            {"destruct", p_get_destruct},
            {"refresh", p_refresh},
            {"reposition", p_get_reposition},
            {"clear", p_get_clear},
            {"setInterval", p_set_interval},
            {"placeholder", p_get_placeholder},
            {"hint", p_get_hint},
            {"valid", p_is_valid},
            {"caret", p_get_caret},
            {"text", p_get_text},
            {"editable", p_get_editable},
            {"edited", p_get_edited},
            {"lineNumbers", p_get_line_numbers},
            {"lineAt", p_get_line_at},
            {"linePos", p_get_line_pos},
            {"syntax", p_get_syntax},
            {"markup", p_get_markup},
            {"src", p_get_src},
            {"value", p_get_value},
            {"min", p_get_min},
            {"max", p_get_max},
            {"step", p_get_step},
            {"scroll", p_get_scroll},
            {"trackWidth", p_get_track_width},
            {"trackColor", p_get_track_color},
            {"textColor", p_get_text_color},
            {"checked", p_is_checked},
            {"page", p_get_page},
            {"back", p_get_back},
            {"reset", p_get_reset},
            {"paste", p_get_paste},
            {"inventory", p_get_inventory},
            {"focused", p_get_focused},
            {"cursor", p_get_cursor},
            {"data", p_get_data},
            {"parent", p_get_parent},
            {"region", p_get_region},
            {"options", p_get_options},
        };
    auto func = getters.find(attr);
    if (func != getters.end()) {
        auto docnode = get_document_node_impl(L, docname, element);
        auto node = docnode.node;
        return func->second(node.get(), L);
    }
    return 0;
}

static void p_set_color(UINode* node, lua::State* L, int idx) {
    node->setColor(lua::tocolor(L, idx));
}
static void p_set_hover_color(UINode* node, lua::State* L, int idx) {
    node->setHoverColor(lua::tocolor(L, idx));
}
static void p_set_pressed_color(UINode* node, lua::State* L, int idx) {
    node->setPressedColor(lua::tocolor(L, idx));
}
static void p_set_tooltip(UINode* node, lua::State* L, int idx) {
    node->setTooltip(lua::require_wstring(L, idx));
}
static void p_set_tooltip_delay(UINode* node, lua::State* L, int idx) {
    node->setTooltipDelay(lua::tonumber(L, idx));
}
static void p_set_pos(UINode* node, lua::State* L, int idx) {
    node->setPos(lua::tovec2(L, idx));
}
static void p_set_wpos(UINode* node, lua::State* L, int idx) {
    node->setPos(lua::tovec2(L, idx) - node->calcPos());
}
static void p_set_size(UINode* node, lua::State* L, int idx) {
    node->setSize(lua::tovec2(L, idx));
}
static void p_set_interactive(UINode* node, lua::State* L, int idx) {
    node->setInteractive(lua::toboolean(L, idx));
}
static void p_set_visible(UINode* node, lua::State* L, int idx) {
    node->setVisible(lua::toboolean(L, idx));
}
static void p_set_enabled(UINode* node, lua::State* L, int idx) {
    node->setEnabled(lua::toboolean(L, idx));
}
static void p_set_placeholder(UINode* node, lua::State* L, int idx) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        box->setPlaceholder(lua::require_wstring(L, idx));
    }
}
static void p_set_hint(UINode* node, lua::State* L, int idx) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        box->setHint(lua::require_wstring(L, idx));
    }
}
static void p_set_text(UINode* node, lua::State* L, int idx) {
    if (auto label = dynamic_cast<Label*>(node)) {
        label->setText(lua::require_wstring(L, idx));
    } else if (auto button = dynamic_cast<Button*>(node)) {
        button->setText(lua::require_wstring(L, idx));
    } else if (auto box = dynamic_cast<TextBox*>(node)) {
        box->setText(lua::require_wstring(L, idx));
    }
}
static void p_set_caret(UINode* node, lua::State* L, int idx) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        box->setCaret(static_cast<ptrdiff_t>(lua::tointeger(L, idx)));
    }
}
static void p_set_editable(UINode* node, lua::State* L, int idx) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        box->setEditable(lua::toboolean(L, idx));
    }
}
static void p_set_edited(UINode* node, lua::State* L, int idx) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        if (!lua::toboolean(L, idx)) {
            box->setUnedited();
        }
    }
}
static void p_set_line_numbers(UINode* node, lua::State* L, int idx) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        box->setShowLineNumbers(lua::toboolean(L, idx));
    }
}
static void p_set_syntax(UINode* node, lua::State* L, int idx) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        box->setSyntax(lua::require_string(L, idx));
    }
}
static void p_set_markup(UINode* node, lua::State* L, int idx) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        box->setMarkup(lua::require_string(L, idx));
    } else if (auto label = dynamic_cast<Label*>(node)) {
        label->setMarkup(lua::require_string(L, idx));
    }
}
static void p_set_src(UINode* node, lua::State* L, int idx) {
    if (auto image = dynamic_cast<Image*>(node)) {
        image->setTexture(lua::require_string(L, idx));
    } else if (auto iframe = dynamic_cast<InlineFrame*>(node)) {
        iframe->setSrc(lua::require_string(L, idx));
    } else if (auto modelviewer = dynamic_cast<ModelViewer*>(node)) {
        modelviewer->setModel(lua::require_string(L, idx));
    }
}
static void p_set_region(UINode* node, lua::State* L, int idx) {
    if (auto image = dynamic_cast<Image*>(node)) {
        auto vec = lua::tovec4(L, idx);
        image->setRegion(UVRegion(vec.x, vec.y, vec.z, vec.w));
    }
}
static void p_set_options(UINode* node, lua::State* L, int idx) {
    if (auto selectbox = dynamic_cast<SelectBox*>(node)) {
        if (!lua::istable(L, idx)) {
            throw std::runtime_error("options table expected");
        }
        std::vector<SelectBox::Option> options;
        size_t size = lua::objlen(L, idx);
        for (size_t i = 0; i < size; i++) {
            lua::rawgeti(L, i + 1, idx);
            
            SelectBox::Option option;

            lua::getfield(L, "value");
            option.value = lua::require_string(L, -1);
            lua::pop(L);

            lua::getfield(L, "text");
            option.text = lua::require_wstring(L, -1);
            lua::pop(L, 2);
            
            options.push_back(std::move(option));
        }
        selectbox->setOptions(std::move(options));
    }
}
static void p_set_value(UINode* node, lua::State* L, int idx) {
    if (auto bar = dynamic_cast<TrackBar*>(node)) {
        bar->setValue(lua::tonumber(L, idx));
    } else if (auto selectbox = dynamic_cast<SelectBox*>(node)) {
        auto value = lua::require_lstring(L, idx);
        const auto& options = selectbox->getOptions();
        for (const auto& option : options) {
            if (option.value == value) {
                selectbox->setSelected(option);
                return;
            }
        }
        selectbox->setSelected(SelectBox::Option {
            std::string(value), util::str2wstr_utf8(value)});
    }
}
static void p_set_min(UINode* node, lua::State* L, int idx) {
    if (auto bar = dynamic_cast<TrackBar*>(node)) {
        bar->setMin(lua::tonumber(L, idx));
    }
}
static void p_set_max(UINode* node, lua::State* L, int idx) {
    if (auto bar = dynamic_cast<TrackBar*>(node)) {
        bar->setMax(lua::tonumber(L, idx));
    }
}
static void p_set_step(UINode* node, lua::State* L, int idx) {
    if (auto bar = dynamic_cast<TrackBar*>(node)) {
        bar->setStep(lua::tonumber(L, idx));
    }
}
static void p_set_track_width(UINode* node, lua::State* L, int idx) {
    if (auto bar = dynamic_cast<TrackBar*>(node)) {
        bar->setTrackWidth(lua::tointeger(L, idx));
    }
}
static void p_set_track_color(UINode* node, lua::State* L, int idx) {
    if (auto bar = dynamic_cast<TrackBar*>(node)) {
        bar->setTrackColor(lua::tocolor(L, idx));
    }
}
static void p_set_text_color(UINode* node, lua::State* L, int idx) {
    if (auto box = dynamic_cast<TextBox*>(node)) {
        box->setTextColor(lua::tocolor(L, idx));
    }
}
static void p_set_checked(UINode* node, lua::State* L, int idx) {
    if (auto box = dynamic_cast<CheckBox*>(node)) {
        box->setChecked(lua::toboolean(L, idx));
    } else if (auto box = dynamic_cast<FullCheckBox*>(node)) {
        box->setChecked(lua::toboolean(L, idx));
    }
}
static void p_set_page(UINode* node, lua::State* L, int idx) {
    if (auto menu = dynamic_cast<Menu*>(node)) {
        menu->setPage(lua::require_string(L, idx));
    }
}
static void p_set_inventory(UINode* node, lua::State* L, int idx) {
    if (auto view = dynamic_cast<InventoryView*>(node)) {
        auto inventory = level->inventories->get(lua::tointeger(L, idx));
        if (inventory == nullptr) {
            view->unbind();
        } else {
            view->bind(inventory, content);
        }
    }
}
static void p_set_focused(
    UINode* node, lua::State* L, int idx
) {
    if (lua::toboolean(L, idx) && !node->isFocused()) {
        engine->getGUI().setFocus(node->shared_from_this());
    } else if (node->isFocused()) {
        node->defocus();
    }
}

static void p_set_cursor(UINode* node, lua::State* L, int idx) {
    auto cursor = CursorShape::ARROW; // reset to default
    CursorShapeMeta.getItem(lua::require_string(L, idx), cursor);
    node->setCursor(cursor);
}

static int p_set_scroll(UINode* node, lua::State* L, int idx) {
    if (auto container = dynamic_cast<Container*>(node)) {
        container->setScroll(lua::tointeger(L, idx));
    }
    return 0;
}

static int l_gui_setattr(lua::State* L) {
    if (!lua::isstring(L, 1)) {
        throw std::runtime_error("document name is not a string");
    }
    if (!lua::isstring(L, 2)) {
        throw std::runtime_error("element id is not a string");
    }
    if (!lua::isstring(L, 3)) {
        throw std::runtime_error("attribute name is not a string");
    }
    auto docname = lua::require_string(L, 1);
    auto element = lua::require_string(L, 2);
    auto attr = lua::require_string(L, 3);

    auto docnode = get_document_node_impl(L, docname, element);
    auto node = docnode.node;

    static const std::unordered_map<
        std::string_view,
        std::function<void(UINode*, lua::State*, int)>>
        setters {
            {"color", p_set_color},
            {"hoverColor", p_set_hover_color},
            {"pressedColor", p_set_pressed_color},
            {"tooltip", p_set_tooltip},
            {"tooltipDelay", p_set_tooltip_delay},
            {"pos", p_set_pos},
            {"wpos", p_set_wpos},
            {"size", p_set_size},
            {"interactive", p_set_interactive},
            {"visible", p_set_visible},
            {"enabled", p_set_enabled},
            {"placeholder", p_set_placeholder},
            {"hint", p_set_hint},
            {"text", p_set_text},
            {"editable", p_set_editable},
            {"edited", p_set_edited},
            {"lineNumbers", p_set_line_numbers},
            {"syntax", p_set_syntax},
            {"markup", p_set_markup},
            {"src", p_set_src},
            {"caret", p_set_caret},
            {"value", p_set_value},
            {"min", p_set_min},
            {"max", p_set_max},
            {"step", p_set_step},
            {"scroll", p_set_scroll},
            {"trackWidth", p_set_track_width},
            {"trackColor", p_set_track_color},
            {"textColor", p_set_text_color},
            {"checked", p_set_checked},
            {"page", p_set_page},
            {"inventory", p_set_inventory},
            {"cursor", p_set_cursor},
            {"focused", p_set_focused},
            {"region", p_set_region},
            {"options", p_set_options},
        };
    auto func = setters.find(attr);
    if (func != setters.end()) {
        func->second(node.get(), L, 4);
    }
    return 0;
}

static int l_gui_get_env(lua::State* L) {
    auto name = lua::require_string(L, 1);
    auto doc = engine->getAssets()->get<UiDocument>(name);
    if (doc == nullptr) {
        throw std::runtime_error(
            "document '" + std::string(name) + "' not found"
        );
    }
    lua::getglobal(L, lua::env_name(*doc->getEnvironment()));
    return 1;
}

static int l_gui_str(lua::State* L) {
    auto text = lua::require_wstring(L, 1);
    if (!lua::isnoneornil(L, 2)) {
        auto context = lua::require_wstring(L, 2);
        lua::pushwstring(L, langs::get(text, context));
    } else {
        lua::pushwstring(L, langs::get(text));
    }
    return 1;
}

static int l_gui_reindex(lua::State* L) {
    auto name = lua::require_string(L, 1);
    auto doc = engine->getAssets()->get<UiDocument>(name);
    if (doc == nullptr) {
        throw std::runtime_error(
            "document '" + std::string(name) + "' not found"
        );
    }
    doc->rebuildIndices();
    return 0;
}

/// @brief gui.get_locales_info() -> table of tables
static int l_gui_get_locales_info(lua::State* L) {
    auto& locales = langs::get_locales_info();
    lua::createtable(L, 0, locales.size());
    for (auto& entry : locales) {
        lua::createtable(L, 0, 1);
        lua::pushstring(L, entry.second.name);
        lua::setfield(L, "name");
        lua::setfield(L, entry.first);
    }
    return 1;
}

static int l_gui_getviewport(lua::State* L) {
    return lua::pushvec2(L, engine->getGUI().getContainer()->getSize());
}

static int l_gui_clear_markup(lua::State* L) {
    auto lang = lua::require_string(L, 1);
    std::string text = lua::require_string(L, 2);
    if (std::strcmp(lang, "md") == 0) {
        auto [processed, _] = markdown::process(text, true);
        text = std::move(processed);
    }
    return lua::pushstring(L, text);
}

static int l_gui_escape_markup(lua::State* L) {
    auto lang = lua::require_string(L, 1);
    std::string text = lua::require_string(L, 2);
    if (std::strcmp(lang, "md") == 0) {
        text = markdown::escape<char>(text);
    }
    return lua::pushstring(L, text);
}

static int l_gui_confirm(lua::State* L) {
    auto question = lua::require_wstring(L, 1);
    runnable onconfirm = nullptr;
    if (lua::gettop(L) >= 2) {
        lua::pushvalue(L, 2);
        onconfirm = lua::create_runnable(L);
    }
    runnable ondeny = nullptr;
    if (lua::gettop(L) >= 3) {
        lua::pushvalue(L, 3);
        ondeny = lua::create_runnable(L);
    }
    std::wstring yestext = L"";
    if (lua::gettop(L) >= 4) {
        yestext = lua::require_wstring(L, 4);
    }
    std::wstring notext = L"";
    if (lua::gettop(L) >= 5) {
        notext = lua::require_wstring(L, 5);
    }
    guiutil::confirm(*engine, question, onconfirm, ondeny, yestext, notext);
    return 0;
}

static int l_gui_alert(lua::State* L) {
    auto message = lua::require_wstring(L, 1);
    runnable onconfirm = nullptr;
    if (lua::gettop(L) >= 2) {
        lua::pushvalue(L, 2);
        onconfirm = lua::create_runnable(L);
    }
    guiutil::alert(*engine, message, onconfirm);
    return 0;
}

static int l_gui_load_document(lua::State* L) {
    io::path filename = lua::require_string(L, 1);
    auto alias = lua::require_string(L, 2);
    auto args = lua::tovalue(L, 3);
    
    auto documentPtr = UiDocument::read(
        engine->getGUI(),
        scripting::get_root_environment(),
        alias,
        filename,
        filename.string()
    );
    auto document = documentPtr.get();
    engine->getAssets()->store(std::move(documentPtr), alias);
    scripting::on_ui_open(document, {args});
    return 0;
}

const luaL_Reg guilib[] = {
    {"get_viewport", lua::wrap<l_gui_getviewport>},
    {"getattr", lua::wrap<l_gui_getattr>},
    {"setattr", lua::wrap<l_gui_setattr>},
    {"get_env", lua::wrap<l_gui_get_env>},
    {"str", lua::wrap<l_gui_str>},
    {"get_locales_info", lua::wrap<l_gui_get_locales_info>},
    {"clear_markup", lua::wrap<l_gui_clear_markup>},
    {"escape_markup", lua::wrap<l_gui_escape_markup>},
    {"confirm", lua::wrap<l_gui_confirm>},
    {"alert", lua::wrap<l_gui_alert>},
    {"load_document", lua::wrap<l_gui_load_document>},
    {"__reindex", lua::wrap<l_gui_reindex>},
    {NULL, NULL}
};
