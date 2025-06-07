#define VC_ENABLE_REFLECTION
#include "gui_xml.hpp"

#include <stdexcept>
#include <utility>

#include "GUI.hpp"
#include "elements/Button.hpp"
#include "elements/Canvas.hpp"
#include "elements/CheckBox.hpp"
#include "elements/TextBox.hpp"
#include "elements/SplitBox.hpp"
#include "elements/TrackBar.hpp"
#include "elements/Image.hpp"
#include "elements/InlineFrame.hpp"
#include "elements/InputBindBox.hpp"
#include "elements/InventoryView.hpp"
#include "elements/Menu.hpp"
#include "elements/Panel.hpp"
#include "elements/TextBox.hpp"
#include "elements/TrackBar.hpp"
#include "elements/ModelViewer.hpp"
#include "engine/Engine.hpp"
#include "frontend/locale.hpp"
#include "frontend/menu.hpp"
#include "items/Inventory.hpp"
#include "logic/scripting/scripting.hpp"
#include "maths/voxmaths.hpp"
#include "util/stringutil.hpp"

using namespace gui;

static Align align_from_string(const std::string& str, Align def) {
    if (str == "left") return Align::left;
    if (str == "center") return Align::center;
    if (str == "right") return Align::right;
    if (str == "top") return Align::top;
    if (str == "bottom") return Align::bottom;
    return def;
}

static Gravity gravity_from_string(const std::string& str) {
    static const std::unordered_map<std::string, Gravity> gravity_names {
        {"top-left", Gravity::top_left},
        {"top-center", Gravity::top_center},
        {"top-right", Gravity::top_right},
        {"center-left", Gravity::center_left},
        {"center-center", Gravity::center_center},
        {"center-right", Gravity::center_right},
        {"bottom-left", Gravity::bottom_left},
        {"bottom-center", Gravity::bottom_center},
        {"bottom-right", Gravity::bottom_right},
    };
    auto found = gravity_names.find(str);
    if (found != gravity_names.end()) {
        return found->second;
    }
    return Gravity::none;
}

static runnable create_runnable(
    const UiXmlReader& reader,
    const xml::xmlelement& element,
    const std::string& name
) {
    if (element.has(name)) {
        const std::string& text = element.attr(name).getText();
        if (!text.empty()) {
            return scripting::create_runnable(
                reader.getEnvironment(), text, reader.getFilename()
            );
        }
    }
    return nullptr;
}

static onaction create_action(
    const UiXmlReader& reader,
    const xml::xmlelement& element,
    const std::string& name
) {
    auto callback = create_runnable(reader, element, name);
    if (callback == nullptr) {
        return nullptr;
    }
    return [callback](GUI&) { callback(); };
}

/// @brief Read basic UINode properties
static void read_uinode(
    const UiXmlReader& reader, const xml::xmlelement& element, UINode& node
) {
    if (element.has("id")) {
        node.setId(element.attr("id").getText());
    }
    if (element.has("pos")) {
        node.setPos(element.attr("pos").asVec2());
    }
    if (element.has("min-size")) {
        node.setMinSize(element.attr("min-size").asVec2());
    }
    if (element.has("size")) {
        node.setSize(element.attr("size").asVec2());
    }
    if (element.has("color")) {
        glm::vec4 color = element.attr("color").asColor();
        glm::vec4 hoverColor = color;
        glm::vec4 pressedColor = color;
        if (element.has("hover-color")) {
            hoverColor = node.getHoverColor();
        }
        if (element.has("pressed-color")) {
            pressedColor = node.getPressedColor();
        }
        node.setColor(color);
        node.setHoverColor(hoverColor);
        node.setPressedColor(pressedColor);
    }
    if (element.has("margin")) {
        node.setMargin(element.attr("margin").asVec4());
    }
    if (element.has("z-index")) {
        node.setZIndex(element.attr("z-index").asInt());
    }
    if (element.has("interactive")) {
        node.setInteractive(element.attr("interactive").asBool());
    }
    if (element.has("visible")) {
        node.setVisible(element.attr("visible").asBool());
    }
    if (element.has("enabled")) {
        node.setEnabled(element.attr("enabled").asBool());
    }
    if (element.has("position-func")) {
        node.setPositionFunc(scripting::create_vec2_supplier(
            reader.getEnvironment(),
            element.attr("position-func").getText(),
            reader.getFilename()
        ));
    }
    if (element.has("size-func")) {
        node.setSizeFunc(scripting::create_vec2_supplier(
            reader.getEnvironment(),
            element.attr("size-func").getText(),
            reader.getFilename()
        ));
    }
    if (element.has("hover-color")) {
        node.setHoverColor(element.attr("hover-color").asColor());
    }
    if (element.has("pressed-color")) {
        node.setPressedColor(element.attr("pressed-color").asColor());
    }
    std::string alignName = element.attr("align", "").getText();
    node.setAlign(align_from_string(alignName, node.getAlign()));

    if (element.has("gravity")) {
        node.setGravity(gravity_from_string(element.attr("gravity").getText()));
    }

    if (element.has("tooltip")) {
        auto tooltip = util::str2wstr_utf8(element.attr("tooltip").getText());
        if (!tooltip.empty() && tooltip[0] == '@') {
            tooltip = langs::get(
                tooltip.substr(1), util::str2wstr_utf8(reader.getContext())
            );
        }
        node.setTooltip(tooltip);
    }
    if (element.has("tooltip-delay")) {
        node.setTooltipDelay(element.attr("tooltip-delay").asFloat());
    }
    if (element.has("cursor")) {
        CursorShape cursor;
        if (CursorShapeMeta.getItem(element.attr("cursor").getText(), cursor)) {
            node.setCursor(cursor);
        }
    }

    if (auto onclick = create_action(reader, element, "onclick")) {
        node.listenAction(onclick);
    }

    if (auto onfocus = create_action(reader, element, "onfocus")) {
        node.listenFocus(onfocus);
    }

    if (auto ondefocus = create_action(reader, element, "ondefocus")) {
        node.listenDefocus(ondefocus);
    }

    if (auto ondoubleclick = create_action(reader, element, "ondoubleclick")) {
        node.listenDoubleClick(ondoubleclick);
    }
}

static void read_container_impl(
    UiXmlReader& reader, const xml::xmlelement& element, Container& container
) {
    read_uinode(reader, element, container);

    if (element.has("scrollable")) {
        container.setScrollable(element.attr("scrollable").asBool());
    }
    if (element.has("scroll-step")) {
        container.setScrollStep(element.attr("scroll-step").asInt());
    }
    for (auto& sub : element.getElements()) {
        if (sub->isText()) continue;
        auto subnode = reader.readUINode(*sub);
        if (subnode) {
            container.add(subnode);
        }
    }
}

void UiXmlReader::readUINode(
    UiXmlReader& reader, const xml::xmlelement& element, Container& container
) {
    read_container_impl(reader, element, container);
}

void UiXmlReader::readUINode(
    const UiXmlReader& reader, const xml::xmlelement& element, UINode& node
) {
    read_uinode(reader, element, node);
}

static void read_base_panel_impl(
    UiXmlReader& reader,
    const xml::xmlelement& element,
    BasePanel& panel
) {
    read_uinode(reader, element, panel);

    if (element.has("padding")) {
        glm::vec4 padding = element.attr("padding").asVec4();
        panel.setPadding(padding);
        glm::vec2 size = panel.getSize();
        panel.setSize(glm::vec2(
            size.x + padding.x + padding.z, size.y + padding.y + padding.w
        ));
    }
    if (element.has("orientation")) {
        auto &oname = element.attr("orientation").getText();
        if (oname == "horizontal") {
            panel.setOrientation(Orientation::horizontal);
        }
    }
}

static void read_panel_impl(
    UiXmlReader& reader,
    const xml::xmlelement& element,
    Panel& panel,
    bool subnodes = true
) {
    read_base_panel_impl(reader, element, panel);

    if (element.has("size")) {
        panel.setResizing(false);
    }
    if (element.has("max-length")) {
        panel.setMaxLength(element.attr("max-length").asInt());
    }
    if (element.has("min-length")) {
        panel.setMinLength(element.attr("min-length").asInt());
    }
    if (element.has("orientation")) {
        auto& oname = element.attr("orientation").getText();
        if (oname == "horizontal") {
            panel.setOrientation(Orientation::horizontal);
        }
    }
    if (subnodes) {
        for (auto& sub : element.getElements()) {
            if (sub->isText()) continue;
            auto subnode = reader.readUINode(*sub);
            if (subnode) {
                panel.add(subnode);
            }
        }
    }
}

static std::wstring parse_inner_text(
    const xml::xmlelement& element, const std::string& context
) {
    std::wstring text = L"";
    if (element.size() == 1) {
        std::string source = element.sub(0).getInnerText();
        util::trim(source);
        text = util::str2wstr_utf8(source);
        if (text[0] == '@') {
            if (context.empty()) {
                text = langs::get(text.substr(1));
            } else {
                text = langs::get(text.substr(1), util::str2wstr_utf8(context));
            }
        }
    }
    return text;
}

static std::shared_ptr<UINode> read_label(
    const UiXmlReader& reader, const xml::xmlelement& element
) {
    std::wstring text = parse_inner_text(element, reader.getContext());
    auto label = std::make_shared<Label>(reader.getGUI(), text);
    read_uinode(reader, element, *label);
    if (element.has("valign")) {
        label->setVerticalAlign(align_from_string(
            element.attr("valign").getText(), label->getVerticalAlign()
        ));
    }
    if (element.has("supplier")) {
        label->textSupplier(scripting::create_wstring_supplier(
            reader.getEnvironment(),
            element.attr("supplier").getText(),
            reader.getFilename()
        ));
    }
    if (element.has("autoresize")) {
        label->setAutoResize(element.attr("autoresize").asBool());
    }
    if (element.has("multiline")) {
        label->setMultiline(element.attr("multiline").asBool());
        if (!element.has("valign")) {
            label->setVerticalAlign(Align::top);
        }
    }
    if (element.has("text-wrap")) {
        label->setTextWrapping(element.attr("text-wrap").asBool());
    }
    if (element.has("markup")) {
        label->setMarkup(element.attr("markup").getText());
    }
    return label;
}

static std::shared_ptr<UINode> read_container(
    UiXmlReader& reader, const xml::xmlelement& element
) {
    auto container = std::make_shared<Container>(reader.getGUI(), glm::vec2());
    read_container_impl(reader, element, *container);
    return container;
}

static std::shared_ptr<UINode> read_split_box(
    UiXmlReader& reader, const xml::xmlelement& element
) {
    float splitPos = element.attr("split-pos", "0.5").asFloat();
    Orientation orientation =
        element.attr("orientation", "vertical").getText() == "horizontal"
            ? Orientation::horizontal
            : Orientation::vertical;
    auto splitBox = std::make_shared<SplitBox>(
        reader.getGUI(), glm::vec2(), splitPos, orientation
    );
    read_base_panel_impl(reader, element, *splitBox);
    for (auto& sub : element.getElements()) {
        if (sub->isText())
            continue;
        auto subnode = reader.readUINode(*sub);
        if (subnode) {
            splitBox->add(subnode);
        }
    }
    return splitBox;
}

static std::shared_ptr<UINode> read_model_viewer(
    UiXmlReader& reader, const xml::xmlelement& element
) {
    auto model = element.attr("src", "").getText();
    auto viewer = std::make_shared<ModelViewer>(
        reader.getGUI(), glm::vec2(), model
    );
    read_container_impl(reader, element, *viewer);
    if (element.has("center")) {
        viewer->setCenter(element.attr("center").asVec3());
    }
    if (element.has("cam-rotation")) {
        viewer->setRotation(glm::radians(element.attr("cam-rotation").asVec3()));
    }
    return viewer;
}

static std::shared_ptr<UINode> read_panel(
    UiXmlReader& reader, const xml::xmlelement& element
) {
    float interval = element.attr("interval", "2").asFloat();
    auto panel = std::make_shared<Panel>(
        reader.getGUI(), glm::vec2(), glm::vec4(), interval
    );
    read_panel_impl(reader, element, *panel);
    return panel;
}

static std::shared_ptr<UINode> read_button(
    UiXmlReader& reader, const xml::xmlelement& element
) {
    auto& gui = reader.getGUI();
    glm::vec4 padding = element.attr("padding", "10").asVec4();

    std::shared_ptr<Button> button;
    auto& elements = element.getElements();
    if (!elements.empty() && !elements[0]->isText()) {
        auto inner = reader.readUINode(*elements.at(0));
        if (inner != nullptr) {
            button = std::make_shared<Button>(gui, inner, padding);
        } else {
            button = std::make_shared<Button>(gui, L"", padding, nullptr);
        }
        read_panel_impl(reader, element, *button, false);
    } else {
        std::wstring text = parse_inner_text(element, reader.getContext());
        button = std::make_shared<Button>(gui, text, padding, nullptr);
        read_panel_impl(reader, element, *button, true);
    }
    if (element.has("text-align")) {
        button->setTextAlign(align_from_string(
            element.attr("text-align").getText(), button->getTextAlign()
        ));
    }
    return button;
}

static std::shared_ptr<UINode> read_check_box(
    UiXmlReader& reader, const xml::xmlelement& element
) {
    auto text = parse_inner_text(element, reader.getContext());
    bool checked = element.attr("checked", "false").asBool();
    auto checkbox = std::make_shared<FullCheckBox>(
        reader.getGUI(), text, glm::vec2(32), checked
    );
    read_panel_impl(reader, element, *checkbox);

    if (element.has("consumer")) {
        checkbox->setConsumer(scripting::create_bool_consumer(
            reader.getEnvironment(),
            element.attr("consumer").getText(),
            reader.getFilename()
        ));
    }

    if (element.has("supplier")) {
        checkbox->setSupplier(scripting::create_bool_supplier(
            reader.getEnvironment(),
            element.attr("supplier").getText(),
            reader.getFilename()
        ));
    }
    return checkbox;
}

static std::shared_ptr<UINode> read_text_box(
    UiXmlReader& reader, const xml::xmlelement& element
) {
    auto placeholder =
        util::str2wstr_utf8(element.attr("placeholder", "").getText());
    auto hint = util::str2wstr_utf8(element.attr("hint", "").getText());
    if (!hint.empty() && hint[0] == '@') {
        hint = langs::get(
            hint.substr(1), util::str2wstr_utf8(reader.getContext())
        );
    }
    auto text = parse_inner_text(element, reader.getContext());
    auto textbox = std::make_shared<TextBox>(
        reader.getGUI(), placeholder, glm::vec4(0.0f)
    );
    textbox->setHint(hint);

    read_container_impl(reader, element, *textbox);
    if (element.has("padding")) {
        glm::vec4 padding = element.attr("padding").asVec4();
        textbox->setPadding(padding);
        glm::vec2 size = textbox->getSize();
        textbox->setSize(glm::vec2(
            size.x + padding.x + padding.z, size.y + padding.y + padding.w
        ));
    }
    textbox->setText(text);

    if (element.has("syntax")) {
        textbox->setSyntax(element.attr("syntax").getText());
    }
    if (element.has("multiline")) {
        textbox->setMultiline(element.attr("multiline").asBool());
    }
    if (element.has("text-wrap")) {
        textbox->setTextWrapping(element.attr("text-wrap").asBool());
    }
    if (element.has("editable")) {
        textbox->setEditable(element.attr("editable").asBool());
    }
    if (element.has("autoresize")) {
        textbox->setAutoResize(element.attr("autoresize").asBool());
    }
    if (element.has("line-numbers")) {
        textbox->setShowLineNumbers(element.attr("line-numbers").asBool());
    }
    if (element.has("markup")) {
        textbox->setMarkup(element.attr("markup").getText());
    }
    if (element.has("consumer")) {
        textbox->setTextConsumer(scripting::create_wstring_consumer(
            reader.getEnvironment(),
            element.attr("consumer").getText(),
            reader.getFilename()
        ));
    }
    if (element.has("sub-consumer")) {
        textbox->setTextSubConsumer(scripting::create_wstring_consumer(
            reader.getEnvironment(),
            element.attr("sub-consumer").getText(),
            reader.getFilename()
        ));
    }
    if (element.has("supplier")) {
        textbox->setTextSupplier(scripting::create_wstring_supplier(
            reader.getEnvironment(),
            element.attr("supplier").getText(),
            reader.getFilename()
        ));
    }
    if (element.has("focused-color")) {
        textbox->setFocusedColor(element.attr("focused-color").asColor());
    }
    if (element.has("error-color")) {
        textbox->setErrorColor(element.attr("error-color").asColor());
    }
    if (element.has("text-color")) {
        textbox->setTextColor(element.attr("text-color").asColor());
    }
    if (element.has("validator")) {
        textbox->setTextValidator(scripting::create_wstring_validator(
            reader.getEnvironment(),
            element.attr("validator").getText(),
            reader.getFilename()
        ));
    }
    if (element.has("oncontrolkey")) {
        textbox->setOnControlCombination(scripting::create_key_handler(
            reader.getEnvironment(),
            element.attr("oncontrolkey").getText(),
            reader.getFilename()
        ));
    }
    if (auto onUpPressed = create_runnable(reader, element, "onup")) {
        textbox->setOnUpPressed(onUpPressed);
    }
    if (auto onDownPressed = create_runnable(reader, element, "ondown")) {
        textbox->setOnDownPressed(onDownPressed);
    }
    return textbox;
}

static std::shared_ptr<UINode> read_image(
    const UiXmlReader& reader, const xml::xmlelement& element
) {
    std::string src = element.attr("src", "").getText();
    auto image = std::make_shared<Image>(reader.getGUI(), src);
    read_uinode(reader, element, *image);

    if (element.has("region")) {
        auto vec = element.attr("region").asVec4();
        image->setRegion(UVRegion(vec.x, vec.y, vec.z, vec.w));
    }
    return image;
}

static std::shared_ptr<UINode> read_canvas(
    const UiXmlReader& reader, const xml::xmlelement& element
) {
    auto size = glm::uvec2 {32, 32};
    if (element.has("size")) {
        size = element.attr("size").asVec2();
    }
    auto image =
        std::make_shared<Canvas>(reader.getGUI(), ImageFormat::rgba8888, size);
    read_uinode(reader, element, *image);
    return image;
}

static std::shared_ptr<UINode> read_track_bar(
    const UiXmlReader& reader, const xml::xmlelement& element
) {
    const auto& env = reader.getEnvironment();
    const auto& file = reader.getFilename();
    float minv = element.attr("min", "0.0").asFloat();
    float maxv = element.attr("max", "1.0").asFloat();
    float def = element.attr("value", "0.0").asFloat();
    float step = element.attr("step", "1.0").asFloat();
    int trackWidth = element.attr("track-width", "12").asInt();
    auto bar = std::make_shared<TrackBar>(
        reader.getGUI(), minv, maxv, def, step, trackWidth
    );
    read_uinode(reader, element, *bar);
    if (element.has("consumer")) {
        bar->setConsumer(scripting::create_number_consumer(
            env, element.attr("consumer").getText(), file
        ));
    }
    if (element.has("sub-consumer")) {
        bar->setSubConsumer(scripting::create_number_consumer(
            env, element.attr("sub-consumer").getText(), file
        ));
    }
    if (element.has("supplier")) {
        bar->setSupplier(scripting::create_number_supplier(
            env, element.attr("supplier").getText(), file
        ));
    }
    if (element.has("track-color")) {
        bar->setTrackColor(element.attr("track-color").asColor());
    }
    if (element.has("change-on-release")) {
        bar->setChangeOnRelease(element.attr("change-on-release").asBool());
    }
    return bar;
}

static std::shared_ptr<UINode> read_input_bind_box(
    UiXmlReader& reader, const xml::xmlelement& element
) {
    auto bindname = element.attr("binding").getText();
    auto& found = reader.getGUI().getInput().getBindings().require(bindname);
    glm::vec4 padding = element.attr("padding", "6").asVec4();
    auto bindbox =
        std::make_shared<InputBindBox>(reader.getGUI(), found, padding);
    read_panel_impl(reader, element, *bindbox);
    return bindbox;
}

static slotcallback read_slot_func(
    InventoryView* view,
    const UiXmlReader& reader,
    const xml::xmlelement& element,
    const std::string& attr
) {
    auto consumer = scripting::create_int_array_consumer(
        reader.getEnvironment(), element.attr(attr).getText()
    );
    return [=](uint slot, ItemStack&) {
        int args[] {
            static_cast<int>(view->getInventory()->getId()),
            static_cast<int>(slot)};
        consumer(args, 2);
    };
}

static void read_slot(
    InventoryView* view, UiXmlReader& reader, const xml::xmlelement& element
) {
    int index = element.attr("index", "0").asInt();
    bool itemSource = element.attr("item-source", "false").asBool();
    bool taking = element.attr("taking", "true").asBool();
    bool placing = element.attr("placing", "true").asBool();
    SlotLayout layout(
        index, glm::vec2(), true, itemSource, nullptr, nullptr, nullptr
    );
    if (element.has("pos")) {
        layout.position = element.attr("pos").asVec2();
    }
    if (element.has("updatefunc")) {
        layout.updateFunc = read_slot_func(view, reader, element, "updatefunc");
    }
    if (element.has("sharefunc")) {
        layout.shareFunc = read_slot_func(view, reader, element, "sharefunc");
    }
    if (element.has("onrightclick")) {
        layout.rightClick =
            read_slot_func(view, reader, element, "onrightclick");
    }
    layout.taking = taking;
    layout.placing = placing;
    auto slot = view->addSlot(layout);
    reader.readUINode(reader, element, *slot);
    view->add(slot);
}

static void read_slots_grid(
    InventoryView* view,
    const UiXmlReader& reader,
    const xml::xmlelement& element
) {
    int startIndex = element.attr("start-index", "0").asInt();
    int rows = element.attr("rows", "0").asInt();
    int cols = element.attr("cols", "0").asInt();
    int count = element.attr("count", "0").asInt();
    const int slotSize = InventoryView::SLOT_SIZE;
    bool taking = element.attr("taking", "true").asBool();
    bool placing = element.attr("placing", "true").asBool();
    int interval = element.attr("interval", "-1").asInt();
    if (interval < 0) {
        interval = InventoryView::SLOT_INTERVAL;
    }
    int padding = element.attr("padding", "-1").asInt();
    if (padding < 0) {
        padding = interval;
    }
    if (rows == 0) {
        rows = ceildiv(count, cols);
    } else if (cols == 0) {
        cols = ceildiv(count, rows);
    } else if (count == 0) {
        count = rows * cols;
    }
    bool itemSource = element.attr("item-source", "false").asBool();
    SlotLayout layout(
        -1, glm::vec2(), true, itemSource, nullptr, nullptr, nullptr
    );
    if (element.has("pos")) {
        layout.position = element.attr("pos").asVec2();
    }
    if (element.has("updatefunc")) {
        layout.updateFunc = read_slot_func(view, reader, element, "updatefunc");
    }
    if (element.has("sharefunc")) {
        layout.shareFunc = read_slot_func(view, reader, element, "sharefunc");
    }
    if (element.has("onrightclick")) {
        layout.rightClick =
            read_slot_func(view, reader, element, "onrightclick");
    }
    layout.padding = padding;
    layout.taking = taking;
    layout.placing = placing;

    int idx = 0;
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++, idx++) {
            if (idx >= count) {
                return;
            }
            SlotLayout slotLayout = layout;
            slotLayout.index = startIndex + idx;
            slotLayout.position += glm::vec2(
                padding + col * (slotSize + interval),
                padding + (rows - row - 1) * (slotSize + interval)
            );
            auto slot = view->addSlot(slotLayout);
            view->add(slot, slotLayout.position);
        }
    }
}

static std::shared_ptr<UINode> read_inventory(
    UiXmlReader& reader, const xml::xmlelement& element
) {
    auto view = std::make_shared<InventoryView>(reader.getGUI());
    view->setColor(glm::vec4(0.122f, 0.122f, 0.122f, 0.878f));  // TODO: fixme
    reader.addIgnore("slot");
    reader.addIgnore("slots-grid");
    reader.readUINode(reader, element, *view);

    for (auto& sub : element.getElements()) {
        if (sub->getTag() == "slot") {
            read_slot(view.get(), reader, *sub);
        } else if (sub->getTag() == "slots-grid") {
            read_slots_grid(view.get(), reader, *sub);
        }
    }
    return view;
}

static std::shared_ptr<UINode> read_page_box(
    UiXmlReader& reader, const xml::xmlelement& element
) {
    auto& gui = reader.getGUI();
    auto menu = std::make_shared<Menu>(gui);
    menu->setPageLoader(gui.getMenu()->getPageLoader());
    read_container_impl(reader, element, *menu);

    return menu;
}

static std::shared_ptr<UINode> read_iframe(
    UiXmlReader& reader, const xml::xmlelement& element
) {
    auto& gui = reader.getGUI();
    auto iframe = std::make_shared<InlineFrame>(gui);
    read_container_impl(reader, element, *iframe);

    std::string src = element.attr("src", "").getText();
    iframe->setSrc(src);
    return iframe;
}

UiXmlReader::UiXmlReader(gui::GUI& gui, scriptenv&& env) : gui(gui), env(std::move(env)) {
    contextStack.emplace("");
    add("image", read_image);
    add("canvas", read_canvas);
    add("iframe", read_iframe);
    add("label", read_label);
    add("panel", read_panel);
    add("button", read_button);
    add("textbox", read_text_box);
    add("pagebox", read_page_box);
    add("splitbox", read_split_box);
    add("checkbox", read_check_box);
    add("trackbar", read_track_bar);
    add("container", read_container);
    add("bindbox", read_input_bind_box);
    add("modelviewer", read_model_viewer);
    add("inventory", read_inventory);
}

void UiXmlReader::add(const std::string& tag, uinode_reader reader) {
    readers[tag] = std::move(reader);
}

bool UiXmlReader::hasReader(const std::string& tag) const {
    return readers.find(tag) != readers.end();
}

void UiXmlReader::addIgnore(const std::string& tag) {
    ignored.insert(tag);
}

std::shared_ptr<UINode> UiXmlReader::readUINode(const xml::xmlelement& element
) {
    if (element.has("if")) {
        const auto& cond = element.attr("if").getText();
        if (cond.empty() || cond == "false" || cond == "nil") return nullptr;
    }
    if (element.has("ifnot")) {
        const auto& cond = element.attr("ifnot").getText();
        if (!(cond.empty() || cond == "false" || cond == "nil")) return nullptr;
    }

    const std::string& tag = element.getTag();
    auto found = readers.find(tag);
    if (found == readers.end()) {
        if (ignored.find(tag) != ignored.end()) {
            return nullptr;
        }
        throw std::runtime_error("unsupported element '" + tag + "'");
    }

    bool hascontext = element.has("context");
    if (hascontext) {
        contextStack.push(element.attr("context").getText());
    }
    auto node = found->second(*this, element);
    if (hascontext) {
        contextStack.pop();
    }
    return node;
}

std::shared_ptr<UINode> UiXmlReader::readXML(
    const std::string& filename, const std::string& source
) {
    this->filename = filename;
    auto document = xml::parse(filename, source);
    return readUINode(*document->getRoot());
}

std::shared_ptr<UINode> UiXmlReader::readXML(
    const std::string& filename, const xml::xmlelement& root
) {
    this->filename = filename;
    return readUINode(root);
}

const std::string& UiXmlReader::getContext() const {
    return contextStack.top();
}

const std::string& UiXmlReader::getFilename() const {
    return filename;
}

const scriptenv& UiXmlReader::getEnvironment() const {
    return env;
}

gui::GUI& UiXmlReader::getGUI() const {
    return gui;
}
