#include "Menu.hpp"

#include <stdexcept>
#include <utility>

using namespace gui;

Menu::Menu(GUI& gui) : Container(gui, glm::vec2(1)){
}

bool Menu::has(const std::string& name) {
    return pages.find(name) != pages.end() ||
           pageSuppliers.find(name) != pageSuppliers.end();
}

void Menu::addPage(
    const std::string& name, const std::shared_ptr<UINode>& panel, bool temporal
) {
    pages[name] = Page {name, panel, temporal};
}

void Menu::removePage(const std::string& name) {
    pages.erase(name);
}

void Menu::addSupplier(const std::string& name, const supplier<std::shared_ptr<UINode>>& pageSupplier) {
    pageSuppliers[name] = pageSupplier;
}

Page Menu::fetchPage(const std::string& name) {
    auto found = pages.find(name);
    if (found == pages.end()) {
        auto supplier = pageSuppliers.find(name);
        if (supplier == pageSuppliers.end()) {
            if (pagesLoader) {
                return {name, pagesLoader(name), false};
            }
            return {};
        } else {
            return {name, supplier->second(), false};
            // supplied pages caching is not implemented
        }
    } else {
        return found->second;
    }
}

void Menu::setPage(const std::string &name, bool history) {
    Page page = fetchPage(name);
    if (page.panel == nullptr) {
        throw std::runtime_error("no page found");
    }
    setPage(page, history);
}

void Menu::setPage(Page page, bool history) {
    if (current.panel) {
        Container::remove(current.panel.get());
        if (history && !current.temporal) {
            pageStack.push(current);
        }
    }
    current = std::move(page);
    Container::add(current.panel);
    setSize(current.panel->getSize());
}

bool Menu::back() {
    if (pageStack.empty()) {
        return false;
    }
    Page page = pageStack.top();
    pageStack.pop();

    auto updated = fetchPage(page.name);
    if (updated.panel) {
        page.panel = updated.panel;
    }

    setPage(page, false);
    return true;
}

void Menu::setPageLoader(PageLoaderFunc loader) {
    pagesLoader = std::move(loader);
}

PageLoaderFunc Menu::getPageLoader() {
    return pagesLoader;
}

Page& Menu::getCurrent() {
    return current;
}

bool Menu::hasOpenPage() const {
    return current.panel != nullptr;
}

void Menu::clearHistory() {
    pageStack = std::stack<Page>();
}

void Menu::reset() {
    clearHistory();
    if (current.panel) {
        Container::remove(current.panel.get());
        current = Page {"", nullptr};
    }
}
