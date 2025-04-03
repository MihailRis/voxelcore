#pragma once

#include "Container.hpp"

#include <stack>

namespace gui {
    struct Page {
        std::string name;
        std::shared_ptr<UINode> panel;
        bool temporal = false;
    };

    using PageLoaderFunc = std::function<std::shared_ptr<UINode>(const std::string&)>;

    class Menu : public Container {
    protected:
        std::unordered_map<std::string, Page> pages;
        std::stack<Page> pageStack;
        Page current;
        std::unordered_map<std::string, supplier<std::shared_ptr<UINode>>> pageSuppliers;
        PageLoaderFunc pagesLoader = nullptr;
    public:
        explicit Menu(GUI& gui);

        /// @brief Check menu have page or page supplier
        /// @param name page name
        bool has(const std::string& name);

        /// @brief Set current page to specified one.
        /// @param name page or page supplier name
        /// @param history previous page will not be saved in history if false
        void setPage(const std::string &name, bool history=true);
        void setPage(Page page, bool history=true);
        void addPage(
            const std::string& name,
            const std::shared_ptr<UINode>& panel,
            bool temporal = false
        );
        void removePage(const std::string& name);
        Page fetchPage(const std::string& name);

        /// @brief Add page supplier used if page is not found
        /// @param name page name
        /// @param pageSupplier page supplier function
        void addSupplier(
            const std::string& name,
            const supplier<std::shared_ptr<UINode>>& pageSupplier
        );

        /// @brief Page loader is called if accessed page is not found 
        void setPageLoader(PageLoaderFunc loader);

        PageLoaderFunc getPageLoader();

        /// @brief Set page to previous saved in history
        bool back();

        /// @brief Clear pages history
        void clearHistory();

        /// @brief Clear history and remove and set page to null
        void reset();
    
        /// @brief Get current page
        Page& getCurrent();

        bool hasOpenPage() const;
    };
}
