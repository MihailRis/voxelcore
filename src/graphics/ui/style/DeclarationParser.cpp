#include "DeclarationParser.hpp"
#include <iostream>
#include <stdexcept>
#include "util/stringutil.hpp"

DeclarationParser::DeclarationParser(std::string_view file, std::string_view source)
    : BasicParser<char>(file, source) {
    // Включаем поддержку C-подобных комментариев
    this->clikeComment = true;
}

std::vector<style::Declaration> DeclarationParser::parseDeclarations() {
    std::vector<style::Declaration> declarations;

    while (hasNext()) {
        skipWhitespace();
        // Если встречаем '}' или конец, значит блок закончился
        // Для inline стиля это будет просто конец
        if (!hasNext() || peekNoJump() == '}') {
            break;
        }

        // Пытаемся распарсить одну декларацию
        auto decl_opt = parseDeclaration();
        if (decl_opt.has_value()) {
            declarations.push_back(std::move(decl_opt.value()));
        } else {
            // Если не удалось распарсить декларацию, пропускаем до ';'
            // или до конца блока/ввода, чтобы избежать зависания
            std::cerr << "Warning: Could not parse declaration near line " << (line + 1) << "\n";
            while(hasNext() && peekNoJump() != ';' && peekNoJump() != '}') {
                nextChar();
            }
            if (hasNext() && peekNoJump() == ';') {
                nextChar(); // Пропускаем ';'
            }
        }
    }

    return declarations;
}

std::optional<style::Declaration> DeclarationParser::parseDeclaration() {
    try {
        skipWhitespace();
        if (!hasNext() || peekNoJump() == '}') {
            return std::nullopt; // Нечего парсить или конец блока
        }

        // 1. Парсим имя свойства (property name)
        std::string property_name = parseCSSIdentifier();
        if (property_name.empty()) {
            // Пропускаем странный символ
            if(hasNext()) nextChar();
            return std::nullopt;
        }

        skipWhitespace();
        // 2. Ожидаем двоеточие
        expect(':');

        // 3. Парсим значение свойства (пока как строку)
        skipWhitespace();
        size_t start = pos;
        // Читаем до ';' или '}' или конца
        while (hasNext() && peekNoJump() != ';' && peekNoJump() != '}') {
            nextChar();
        }
        std::string property_value_str = std::string(source.substr(start, pos - start));
        // Триммим значение
        size_t first = property_value_str.find_first_not_of(" \t\n\r\f\v");
        size_t last = property_value_str.find_last_not_of(" \t\n\r\f\v");
        if (first != std::string::npos) {
             property_value_str = property_value_str.substr(first, (last - first + 1));
        } else {
             property_value_str.clear(); // Только пробелы
        }

        // 4. Ожидаем точку с запятой или проверяем на конец/'}'
        skipWhitespace();
        if (hasNext() && peekNoJump() == ';') {
            nextChar(); // Пропускаем ';'
        } else if (hasNext() && peekNoJump() != '}') {
             // Если нет ';' и это не конец блока, это ошибка
             throw error("';' expected");
        }
        // Если следующий символ '}' или конец ввода, ';' не обязателен (для последней декларации в блоке)

        // TODO: Здесь нужно будет преобразовать property_value_str в style::value
        style::value property_value = style::value(property_value_str); // Предполагаем конструктор из строки

        return style::Declaration{std::move(property_name), std::move(property_value)};

    } catch (const parsing_error& e) {
        std::cerr << "Declaration parsing error: " << e.what() << "\n";
        return std::nullopt;
    }
}

// Парсит CSS-идентификатор
std::string DeclarationParser::parseCSSIdentifier() {
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
bool DeclarationParser::is_css_identifier_start(char c) const {
    return is_identifier_start(c) || c == '-'; 
}

// Проверяет, является ли символ допустимым для продолжения CSS идентификатора
bool DeclarationParser::is_css_identifier_part(char c) const {
    return is_identifier_part(c) || c == '-'; 
}

// --- Внешняя функция для удобства ---
std::vector<style::Declaration> parseInlineStyle(std::string_view style_content) {
    DeclarationParser parser("inline-style", style_content);
    return parser.parseDeclarations();
}