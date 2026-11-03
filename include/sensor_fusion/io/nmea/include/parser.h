#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

class NmeaParser {
private:
    enum class State : char {
        waiting_char,
        inner_char,
        check_star_char,
        new_row_char
    };

    std::array<char, 82> buffer;
    State state_ = State::waiting_char;
    std::uint8_t checksum = 0;
    std::uint8_t expected_checksum_num = 0;
    std::uint8_t expected_checksum = 0;
    std::size_t size_ = 0;

    static int hex_value( char c) {
        if (c >= '0' && c <= '9') {return c - '0';};
        if (c >= 'A' && c <= 'F') {return c - 'A' + 10;};
        if (c >= 'a' && c <= 'f') {return c - 'a' + 10;};
        return -1;
    }

public:
    std::string_view get_sentence() const {
        //Тому розбирати речення треба одразу після true, а якщо його треба зберегти надовго — тоді вже явно скопіювати в std::string
        // бо view показує допоки валідні дані
        return {buffer.data(), size_};
        // return std::string_view(buffer.data(), size_);
    }
    bool receive_byte(unsigned char el) {
        if (el == '$') {
            size_ = 0;
            checksum = 0;
            state_ = State::inner_char;
            return false;
        }

        switch (state_) {
            case State::waiting_char:
                return false;

            case State::inner_char:
                if (el == '*') {
                    expected_checksum_num = 0;
                    expected_checksum = 0;
                    state_ = State::check_star_char;
                } else if (size_ < buffer.size() && el >= 0x20 && el <= 0x7E) {
                    buffer[size_] = static_cast<char>(el);
                    size_++;
                    checksum ^= el;
                } else {
                    state_ = State::waiting_char;
                }
                return false;

            case State::check_star_char: {
                int value = hex_value(static_cast<char>(el));
                if (value < 0) {
                    state_ = State::waiting_char;
                    return false;
                }
                if (expected_checksum_num == 0) {
                    expected_checksum = static_cast<std::uint8_t>(value * 16);
                } else {
                    expected_checksum = static_cast<std::uint8_t>(expected_checksum + value);
                    state_ = State::new_row_char;
                }
                expected_checksum_num++;
                return false;
            }

            case State::new_row_char:
                if (el == '\r') {
                    return false;
                }
                state_ = State::waiting_char;
                if (el == '\n') {
                    return checksum == expected_checksum;
                }
                return false;
        };
        return false;
    };
};
