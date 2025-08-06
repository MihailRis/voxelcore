#pragma once

#include <algorithm>
#include <cctype>
#include <iostream>
#include <map>
#include <string>
#include <variant>
#include <vector>

#include "Stylesheet.h"
#include "coders/BasicParser.hpp"

class CSSSelectorParser : public BasicParser<char> {
public:
    CSSSelectorParser(
        const std::string_view source, const std::string& filename = ""
    )
        : BasicParser<char>(filename, source) {
        // Включаем поддержку C-стильных комментариев, если они возможны в
        // селекторах
        this->clikeComment = true;
    }

    style::Selector parse();
private:
    std::vector<style::SimpleSelector> parseSimpleSelectorSequence();
    style::SimpleSelector parseSingleSimpleSelector();

    // Парсит идентификатор (имя тега, класса, ID)
    std::string parseIdentifier();

    // Проверяет, является ли символ допустимым для продолжения CSS
    // идентификатора
    bool is_css_identifier_part(char c) const;

    // Парсит комбинатор и возвращает его тип
    style::Combinator parseCombinator();

    // Проверяет, является ли символ комбинатором
    bool is_combinator(char c) const;

    // Парсит псевдокласс и добавляет его к селектору
    void parseAndAddPseudoClass(style::SimpleSelector& selector);

    void parsePseudoClassesForSequence(
        std::vector<style::SimpleSelector>& selectors, bool has_pseudo_prefix
    );
};

// --- Вспомогательная функция для удобства ---
style::Selector parseCSSSelector(std::string_view selector_text);