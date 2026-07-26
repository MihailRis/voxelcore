#include "coders/xml.hpp"
#include "coders/commons.hpp"
#include "api_lua.hpp"

static const char* TAG_ATTR = "#";

static int push_xml(lua::State* L, const xml::xmlelement& elem) {
    if (elem.isText()) {
        return lua::pushlstring(L, elem.getInnerText());
    }

    const auto& attrs = elem.getAttributes();
    const auto& elems = elem.getElements();
    const auto& tag = elem.getTag();

    lua::createtable(L, elem.size(), attrs.size() + 1);
    lua::pushlstring(L, tag);
    lua::setfield(L, TAG_ATTR);

    for (int i = 0; i < elems.size(); i++) {
        push_xml(L, *elems[i]);
        lua::rawseti(L, i + 1);
    }
    
    for (const auto& [key, value] : attrs) {
        lua::pushlstring(L, value.getText());
        lua::setfield(L, key);
    }
    return 1;
}

static std::unique_ptr<xml::xmlelement> toxml(lua::State* L) {
    lua::getfield(L, TAG_ATTR);
    auto tag = lua::require_lstring(L, -1);
    lua::pop(L);
    
    auto elem = std::make_unique<xml::xmlelement>(std::string(tag));
    int length = lua::objlen(L, -1);

    for (int i = 0; i < length; i++) {
        lua::rawgeti(L, i + 1);
        elem->add(toxml(L));
        lua::pop(L);
    }

    lua::pushnil(L);
    while (lua::next(L, -2)) {
        lua::pushvalue(L, -2);
        do {
            if (!lua::isstring(L, -1)) {
                break;
            }
            auto key = lua::require_lstring(L, -1);
            if (key == "@") {
                break;
            }
            auto value = lua::require_lstring(L, -2);
            elem->set(std::string(key), std::string(value));
        } while (false);

        lua::pop(L, 2);
    }
    return elem;
}

static int l_tostring(lua::State* L) {
    bool nice = lua::isboolean(L, 2) ? lua::toboolean(L, 2) : true;
    lua::pushvalue(L, 1);
    auto xml = toxml(L);
    lua::pop(L);
    return lua::pushstring(L, xml::stringify(*xml, nice));
}

static int l_parse(lua::State* L) {
    auto string = lua::require_string(L, 1);
    try {
        auto document = xml::parse("[string]", string);
        return push_xml(L, *document->getRoot());
    } catch (const parsing_error& err) {
        throw err.toRuntimeError();
    }
}

static int l_parse_vcd(lua::State* L) {
    auto string = lua::require_string(L, 1);
    auto rootTag = lua::tostring(L, 2);

    std::unique_ptr<xml::Document> document;
    try {
        document = xml::parse_vcm(
            "[string]", string, rootTag ? rootTag : ""
        );
    } catch (const parsing_error& err) {
        throw err.toRuntimeError();
    }
    const auto& root = *document->getRoot();
    if (rootTag != nullptr) {
        return push_xml(L, root);
    }
    int rootElements = root.size();
    if (rootElements != 1) {
        throw std::runtime_error(
            "one root element expected, got " + std::to_string(rootElements)
        );
    }
    return push_xml(L, *root.getElements()[0]);
}

const luaL_Reg xmllib[] = {
    {"tostring", lua::wrap<l_tostring>},
    {"parse", lua::wrap<l_parse>},
    {"parse_vcd", lua::wrap<l_parse_vcd>},
    {nullptr, nullptr}
};
