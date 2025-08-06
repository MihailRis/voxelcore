#pragma once

#include "coders/BasicParser.hpp"
#include "Stylesheet.h"
#include <vector>
#include <string_view>
#include <optional>


using namespace style;

class DeclarationParser : public BasicParser<char> {
public:
    // Конструктор принимает строку с декларациями (без { и })
    DeclarationParser(std::string_view file, std::string_view source);

    // Парсит список деклараций, разделенных ';'
    // Используется как для блоков правил {...}, так и для inline стилей
    std::vector<style::Declaration> parseDeclarations();

private:
    // Парсит одну декларацию property: value;
    std::optional<style::Declaration> parseDeclaration();

    // Парсит CSS-идентификатор (имя свойства)
    std::string parseCSSIdentifier();

    // Проверяет, является ли символ допустимым для начала/продолжения CSS идентификатора
    bool is_css_identifier_start(char c) const;
    bool is_css_identifier_part(char c) const;
};

// Удобная внешняя функция для парсинга inline стилей
std::vector<style::Declaration> parseInlineStyle(std::string_view style_content);