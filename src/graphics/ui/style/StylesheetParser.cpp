#include "StylesheetParser.hpp"

#include <iostream>
#include <sstream>
#include <stdexcept>

#include "util/stringutil.hpp"

using namespace style;

StylesheetParser::StylesheetParser(
    std::string_view file, std::string_view source
)
    : BasicParser(file, source) {
    clikeComment = true;
}

Stylesheet StylesheetParser::parse() {
    Stylesheet stylesheet;

    while (hasNext()) {
        skipWhitespace();  // Пропускаем ведущие пробелы, комментарии

        // Проверяем, не достигли ли конца
        if (!hasNext()) {
            break;
        }

        // Пытаемся распарсить правило (селекторы { декларации })
        try {
            auto rule_opt = parseRule();
            if (rule_opt.has_value()) {
                stylesheet.rules.push_back(std::move(rule_opt.value()));
            }
            // Если правило не распарсилось (nullopt), parseRule уже обработал
            // ошибку
        } catch (const parsing_error& e) {
            std::cerr << "Error parsing rule: " << e.what() << "\n";
            // Пытаемся восстановиться: пропустить до следующей возможной точки
            // Простая стратегия: пропустить до следующего '}'
            while (hasNext() && peekNoJump() != '}') {
                nextChar();
            }
            if (hasNext() && peekNoJump() == '}') {
                nextChar();  // Пропускаем '}'
            }
            // Продолжаем парсинг
        }
    }

    return stylesheet;
}

std::optional<Rule> StylesheetParser::parseRule() {
    skipWhitespace();
    if (!hasNext()) {
        return std::nullopt;
    }

    // 1. Парсим список селекторов, разделенных запятыми
    auto selectors = parseSelectorList();
    if (selectors.empty()) {
        // Если не удалось распарсить ни одного селектора
        if (!hasNext()) return std::nullopt;

        // Попробуем пропустить непонятный символ и продолжить
        std::cerr << "Warning: No selectors found, skipping character.\n";
        if (hasNext()) nextChar();
        return std::nullopt;
    }

    skipWhitespace();
    // 2. Ожидаем открывающую фигурную скобку
    try {
        expect('{');
    } catch (const parsing_error&) {
        std::cerr << "Error: Expected '{' after selectors.\n";
        // Пытаемся восстановиться
        while (hasNext() && peekNoJump() != '{' && peekNoJump() != '}') {
            nextChar();
        }
        if (hasNext() && peekNoJump() == '{') {
            //nextChar();
        } else {
            return std::nullopt;
        }
    }

    // 3. Парсим декларации
    auto declarations = parseDeclarations();

    skipWhitespace();

    return Rule {std::move(selectors), std::move(declarations)};
}

// Парсит список селекторов, разделенных запятыми
std::vector<Selector> StylesheetParser::parseSelectorList() {
    std::vector<Selector> selectors;
    bool first = true;
    do {
        if (!first) {
            // Пропускаем запятую, если это не первый селектор
            skipWhitespace();
            if (hasNext() && peekNoJump() == ',') {
                nextChar();  // Пропускаем ','
            } else {
                // Если нет запятой, значит, это конец списка селекторов
                break;
            }
        }
        first = false;

        skipWhitespace();

        // Определяем конец текущего селектора: ',' или '{'
        size_t start_pos = pos;
        bool found_delimiter = false;
        while (hasNext() && peekNoJump() != ',' && peekNoJump() != '{') {
            nextChar();
        }
        // pos теперь указывает на ',' или '{' или на конец

        // Извлекаем текст селектора
        std::string selector_text(source.substr(start_pos, pos - start_pos));

        // Триммируем selector_text
        size_t first_char = selector_text.find_first_not_of(" \t\n\r\f\v");
        size_t last_char = selector_text.find_last_not_of(" \t\n\r\f\v");
        if (first_char != std::string::npos) {
            selector_text =
                selector_text.substr(first_char, (last_char - first_char + 1));
        } else {
            selector_text.clear();  // Только пробелы
        }

        if (!selector_text.empty()) {
            try {
                // Предполагается, что функция parseCSSSelector существует и
                // возвращает style::Selector
                selectors.push_back(parseCSSSelector(selector_text));
            } catch (const std::exception& e) {
                std::cerr << "Error parsing selector '" << selector_text
                          << "': " << e.what() << "\n";
            }
        }

    } while (
        hasNext() && peekNoJump() == ','
    );  // Продолжаем, если следующий символ - запятая

    return selectors;
}

std::vector<Declaration> StylesheetParser::parseDeclarations() {
    // На этом моменте позиция парсера должна быть сразу после '{'
    // Нам нужно найти закрывающую '}' и извлечь содержимое между ними.

    size_t block_start_pos = this->pos;  // Позиция начала блока деклараций
    size_t brace_count = 1;              // Мы уже прошли первую '{'

    // Ищем парную закрывающую скобку
    while (hasNext() && brace_count > 0) {
        char c =
            this->source[this->pos];  // Доступ к source напрямую, предполагая,
                                      // что он protected или friend
        if (c == '{') {
            brace_count++;
        } else if (c == '}') {
            brace_count--;
        }
        if (brace_count > 0) {  // Не продвигаем позицию на последней '}'
            this->pos++;
        }
    }

    if (brace_count != 0) {
        throw error("Unmatched '{' in rule block");
    }

    size_t block_end_pos = this->pos;  // Позиция символа '}'

    // Извлекаем подстроку с декларациями
    std::string_view declarations_content =
        this->source.substr(block_start_pos, block_end_pos - block_start_pos);

    // Создаем отдельный парсер для этой подстроки
    // Предполагаем, что у DeclarationParser есть конструктор (std::string_view,
    // std::string_view)
    DeclarationParser decl_parser(this->filename, declarations_content);

    // Парсим декларации
    auto declarations = decl_parser.parseDeclarations();

    // Продвигаем позицию основного парсера за закрывающую скобку
    this->pos = block_end_pos + 1;

    return declarations;
}

std::optional<Declaration> StylesheetParser::parseDeclaration() {
    try {
        skipWhitespace();
        if (!hasNext() || peekNoJump() == '}') {
            return std::nullopt;  // Нечего парсить
        }

        // 1. Парсим имя свойства (property name)
        std::string property_name = parseCSSIdentifier();
        if (property_name.empty()) {
            // Пропускаем странный символ
            if (hasNext()) nextChar();
            return std::nullopt;
        }

        skipWhitespace();
        // 2. Ожидаем двоеточие
        expect(':');

        // 3. Парсим значение свойства (пока как строку)
        // TODO: Здесь нужно будет реализовать полноценный парсер значений CSS
        skipWhitespace();
        size_t start = pos;
        // Читаем до ';' или '}'
        while (hasNext() && peekNoJump() != ';' && peekNoJump() != '}') {
            nextChar();
        }
        std::string property_value_str =
            std::string(source.substr(start, pos - start));
        // Триммим значение
        size_t first = property_value_str.find_first_not_of(" \t\n\r\f\v");
        size_t last = property_value_str.find_last_not_of(" \t\n\r\f\v");
        if (first != std::string::npos) {
            property_value_str =
                property_value_str.substr(first, (last - first + 1));
        } else {
            property_value_str.clear();  // Только пробелы
        }

        // 4. Ожидаем точку с запятой
        skipWhitespace();
        if (hasNext() && peekNoJump() == ';') {
            nextChar();  // Пропускаем ';'
        } else if (hasNext() && peekNoJump() != '}') {
            // Если нет ';' и это не конец блока, это ошибка
            throw error("';' expected");
        }

        // TODO: Здесь нужно будет преобразовать property_value_str в
        // style::value Пока что просто создаем строковое значение
        style::value property_value = style::value(
            property_value_str
        );  // Предполагаем конструктор из строки

        return Declaration {
            std::move(property_name), std::move(property_value)
        };

    } catch (const parsing_error& e) {
        std::cerr << "Declaration parsing error: " << e.what() << "\n";
        return std::nullopt;
    }
}

// Парсит CSS-идентификатор
std::string StylesheetParser::parseCSSIdentifier() {
    if (!hasNext() || !is_css_identifier_start(peekNoJump())) {
        return "";
    }
    size_t start = pos;
    while (hasNext() && is_css_identifier_part(peekNoJump())) {
        nextChar();
    }
    return std::string(source.substr(start, pos - start));
}

// Проверяет, является ли символ допустимым для начала CSS идентификатора
bool StylesheetParser::is_css_identifier_start(char c) const {
    return is_identifier_start(c) ||
           c == '-';  // CSS разрешает '-' в начале (после которого должна быть
                      // буква)
    // Более точная проверка для '-' в начале потребовала бы просмотра
    // следующего символа Для простоты разрешим '-' везде, где разрешены
    // идентификаторы
}

// Проверяет, является ли символ допустимым для продолжения CSS идентификатора
bool StylesheetParser::is_css_identifier_part(char c) const {
    return is_identifier_part(c) ||
           c == '-';  // CSS разрешает '-' в середине/конце
}