#pragma once

/*
 * Токенізатор NMEA-речення.
 *
 * Якщо в рядку n ком, то полів рівно n + 1. Порожнє поле — це string_view
 * довжиною 0, і воно займає свою позицію. Більше ніяких правил немає:
 * токенізатор не обрізає пробіли і не перевіряє вміст.
 *
 * Поля — це вікна на вхідний рядок, без копій. Якщо рядок отримано з
 * NmeaParser::get_buffer(), поля валідні лише до наступного '$' у парсері.
 */

#include <array>
#include <cstddef>
#include <optional>
#include <string_view>

// GSV у NMEA 4.10 має 21 поле; беремо більше
inline constexpr std::size_t kMaxFields = 32;

struct NmeaFields {
public:
    bool push(std::string_view field) {
        if (counter >= fields.size()) {
            return false;
        }
        fields[counter] = field;
        counter++;
        return true;
    }
    std::string_view field(std::size_t index) const {
        if (index >= counter) {
            return {};
        }
        return fields[index];
    }

    std::size_t size() const {
        return counter;
    }

private:
    std::array<std::string_view, kMaxFields> fields{};
    std::size_t counter = 0;
};

inline std::optional<NmeaFields> split_fields(std::string_view sentence) {
    NmeaFields result;

    std::size_t start = 0;
    std::size_t end = sentence.find(',');

    while (end != std::string_view::npos) {
        if (!result.push(sentence.substr(start, end - start))) {
            return std::nullopt;
        }
        start = end + 1;
        end = sentence.find(',', start);
    }

    if (!result.push(sentence.substr(start))) {
        return std::nullopt;
    }
    return result;
}
