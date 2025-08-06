#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "../elements/Node.hpp"
#include "Stylesheet.h"

namespace style {
    class StyleComputer {
    public:
        StyleComputer() = default;

        void set_stylesheets(const std::vector<Stylesheet>& sheets);
        void compute(Node& node);
    private:
        Stylesheet stylesheet;
        void compute_base(Node& node, Node& parent);
        void compute_base(Node& node);

        bool match_selector(Node& node, Selector& selector, Node* parent = nullptr);

        template <typename T>
        bool process_selector_variant( Node& node, const T& selector, Node* parent = nullptr);
        bool process_simple_selector(Node& node, const SimpleSelector& selector);
        bool process_complex_selector(Node& node, ComplexSelector selector, Node* parent = nullptr);
        bool check_pseudo_classes( Node& node, const SimpleSelector& selector );
        bool check_nth_child(const Element& element, const std::string& formula);
        bool check_nth_of_type(const Element& element, const std::string& formula);
        bool check_custom_pseudo_class(const Element& element, const std::string& name);
        bool is_default_state(const ElementState& state);
        ElementState extract_element_state(const Selector& selector);
        void extract_state_from_simple_selector(const SimpleSelector& selector, ElementState& state);
        void extract_state_from_complex_selector(const ComplexSelector& selector, ElementState& state);
    };

}