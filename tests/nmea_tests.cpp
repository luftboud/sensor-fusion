#include "sensor_fusion/io/nmea/include/NmeaFramer.h"
#include "sensor_fusion/io/nmea/include/NmeaTokeniser.h"

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {

int g_failures = 0;

void check(bool ok, const char* text, int line) {
    if (!ok) {
        std::cerr << "line " << line << ": " << text << '\n';
        ++g_failures;
    }
}

#define CHECK(condition) check(static_cast<bool>(condition), #condition, __LINE__)

using Sentences = std::vector<std::string>;

constexpr std::string_view gga_body =
    "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
constexpr std::string_view gga =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
constexpr std::string_view rmc_body =
    "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W";
constexpr std::string_view rmc =
    "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";

constexpr std::size_t max_body_size = 82;

// Sentences are copied on the byte that completes them, because get_sentence()
// is only a view of the parser's buffer.
Sentences collect(NmeaParser& parser, std::string_view input) {
    Sentences sentences;
    for (const char byte : input) {
        if (parser.receive_byte(static_cast<unsigned char>(byte))) {
            sentences.emplace_back(parser.get_sentence());
        }
    }
    return sentences;
}

Sentences collect(std::string_view input) {
    NmeaParser parser;
    return collect(parser, input);
}

bool accepts(std::string_view input, std::string_view expected) {
    const Sentences sentences = collect(input);
    return sentences.size() == 1 && sentences.front() == expected;
}

// A rejected input yields nothing and must not break the sentence after it.
bool rejects(std::string_view input) {
    NmeaParser parser;
    return collect(parser, input).empty() && collect(parser, "$AB*03\r\n") == Sentences{"AB"};
}

// Builds a sentence with a correct checksum, so that a test can reject it for
// another reason. Literal sentences above keep this helper honest.
std::string frame(std::string_view body) {
    constexpr std::string_view digits = "0123456789ABCDEF";

    std::uint8_t checksum = 0;
    for (const char byte : body) {
        checksum ^= static_cast<std::uint8_t>(byte);
    }

    std::string sentence = "$";
    sentence += body;
    sentence += '*';
    sentence += digits[checksum / 16];
    sentence += digits[checksum % 16];
    sentence += "\r\n";
    return sentence;
}

void test_initial_state() {
    const NmeaParser parser{};
    CHECK(parser.get_sentence().empty());
}

void test_valid_sentences() {
    CHECK(accepts("$AB*03\r\n", "AB"));
    CHECK(accepts(gga, gga_body));
    CHECK(accepts(rmc, rmc_body));
}

void test_line_endings() {
    CHECK(accepts("$AB*03\n", "AB"));
    CHECK(accepts("$AB*03\r\n", "AB"));
    CHECK(accepts("$AB*03\r\r\n", "AB"));
    CHECK(collect("$AB*03").empty());
    CHECK(collect("$AB*03\r").empty());
}

void test_hex_case() {
    CHECK(accepts("$AJ*0B\r\n", "AJ"));
    CHECK(accepts("$AJ*0b\r\n", "AJ"));
    CHECK(accepts("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6a\r\n",
                  rmc_body));
}

void test_empty_body_and_fields() {
    CHECK(accepts("$*00\r\n", ""));
    CHECK(accepts("$,,,*2C\r\n", ",,,"));
    CHECK(accepts(frame("GPGGA,,,,,,0,,,,,,,,"), "GPGGA,,,,,,0,,,,,,,,"));
}

void test_true_only_on_last_byte() {
    NmeaParser parser;
    for (std::size_t index = 0; index + 1 < gga.size(); ++index) {
        CHECK(!parser.receive_byte(static_cast<unsigned char>(gga[index])));
    }
    CHECK(parser.receive_byte(static_cast<unsigned char>(gga.back())));
    CHECK(parser.get_sentence() == gga_body);
}

void test_checksum_mismatch() {
    CHECK(rejects("$AB*04\r\n"));
    CHECK(rejects("$AB*13\r\n"));
    CHECK(rejects("$AB*30\r\n"));
    CHECK(rejects("$AB*00\r\n"));
    CHECK(rejects("$AB*FF\r\n"));
    CHECK(rejects("$*01\r\n"));
    CHECK(rejects("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48\r\n"));
}

void test_single_bit_corruption() {
    for (std::size_t index = 0; index < gga.size(); ++index) {
        for (int bit = 0; bit < 8; ++bit) {
            std::string corrupted(gga);
            const int flipped = static_cast<unsigned char>(gga[index]) ^ (1 << bit);
            corrupted[index] = static_cast<char>(flipped);
            CHECK(rejects(corrupted));
        }
    }
}

void test_transposition_is_not_detected() {
    CHECK(accepts("$BA*03\r\n", "BA"));
}

void test_malformed_checksum() {
    CHECK(rejects("$AB*0G\r\n"));
    CHECK(rejects("$AB*G3\r\n"));
    CHECK(rejects("$AB*\r\n"));
    CHECK(rejects("$AB*3\r\n"));
    CHECK(rejects("$AB*033\r\n"));
    CHECK(rejects("$AB**03\r\n"));
    CHECK(rejects("$AB*0 3\r\n"));
}

void test_missing_checksum() {
    CHECK(rejects("$AB\r\n"));
    CHECK(rejects("$AB\n"));
    CHECK(rejects("$AB\r*03\r\n"));
    CHECK(rejects("$AB\n*03\r\n"));
}

void test_junk_after_checksum() {
    CHECK(rejects("$AB*03X\r\n"));
    CHECK(rejects("$AB*03 \r\n"));
    CHECK(rejects("$AB*03\rX\n"));
}

void test_body_byte_range() {
    for (int value = 0; value <= 0xFF; ++value) {
        if (value == '$' || value == '*') {
            continue;
        }

        std::string body = "A";
        body += static_cast<char>(value);
        body += 'B';

        const bool printable = value >= 0x20 && value <= 0x7E;
        CHECK(printable ? accepts(frame(body), body) : rejects(frame(body)));
    }
}

void test_bytes_outside_sentences() {
    CHECK(accepts("garbage$AB*03\r\n", "AB"));
    CHECK(accepts("\r\n\r\n$AB*03\r\n", "AB"));
    CHECK(accepts("\xFF\x80*03\r\n$AB*03\r\n", "AB"));
    CHECK(rejects("AB*03\r\n"));
    CHECK(rejects("*03\r\n"));
    CHECK(rejects("\r\n"));
}

void test_dollar_restarts_sentence() {
    CHECK(accepts("$$AB*03\r\n", "AB"));
    CHECK(accepts("$XY$AB*03\r\n", "AB"));
    CHECK(accepts("$XY*$AB*03\r\n", "AB"));
    CHECK(accepts("$XY*0$AB*03\r\n", "AB"));
    CHECK(accepts("$XY*03$AB*03\r\n", "AB"));
    CHECK(accepts("$XY*03\r$AB*03\r\n", "AB"));
}

void test_length_limit() {
    const std::string longest(max_body_size, 'A');
    const std::string too_long(max_body_size + 1, 'A');
    const std::string far_too_long(max_body_size * 4, 'A');

    CHECK(accepts(frame(longest), longest));
    CHECK(rejects(frame(too_long)));
    CHECK(rejects(frame(far_too_long)));
}

void test_stream_of_sentences() {
    std::string stream = "noise";
    stream += gga;
    stream += "$AB*04\r\n";
    stream += "$AB*03\r\n";
    stream += "$AB*03\n";
    stream += "$AB";
    stream += rmc;

    const Sentences expected = {std::string(gga_body), "AB", "AB", std::string(rmc_body)};
    CHECK(collect(stream) == expected);
}

void test_sentence_survives_until_next_dollar() {
    NmeaParser parser;
    CHECK(collect(parser, "$AB*03\r\n") == Sentences{"AB"});
    CHECK(parser.get_sentence() == "AB");

    CHECK(collect(parser, "noise*\r\n").empty());
    CHECK(parser.get_sentence() == "AB");

    CHECK(collect(parser, "$").empty());
    CHECK(parser.get_sentence().empty());
}

bool splits_into(std::string_view sentence, const std::vector<std::string_view>& expected) {
    const std::optional<NmeaFields> fields = split_fields(sentence);
    if (!fields.has_value() || fields->size() != expected.size()) {
        return false;
    }
    for (std::size_t index = 0; index < expected.size(); ++index) {
        if (fields->field(index) != expected[index]) {
            return false;
        }
    }
    return true;
}

// Joins `count` copies of "X" with commas, giving exactly count - 1 commas.
std::string repeated_fields(std::size_t count) {
    std::string sentence = "X";
    for (std::size_t index = 1; index < count; ++index) {
        sentence += ",X";
    }
    return sentence;
}

void test_split_plain_fields() {
    CHECK(splits_into("A", {"A"}));
    CHECK(splits_into("A,B", {"A", "B"}));
    CHECK(splits_into("GGA,1,2.5,N", {"GGA", "1", "2.5", "N"}));
}

void test_split_empty_fields() {
    CHECK(splits_into("", {""}));
    CHECK(splits_into(",", {"", ""}));
    CHECK(splits_into(",,,", {"", "", "", ""}));
    CHECK(splits_into(",A", {"", "A"}));
    CHECK(splits_into("A,", {"A", ""}));
    CHECK(splits_into("A,,B", {"A", "", "B"}));
}

void test_split_keeps_content_as_is() {
    CHECK(splits_into(" A , B ", {" A ", " B "}));
    CHECK(splits_into("AB*03", {"AB*03"}));
    CHECK(splits_into("$GP,GGA\r\n", {"$GP", "GGA\r\n"}));
}

void test_split_real_sentences() {
    CHECK(splits_into(gga_body, {"GPGGA", "123519", "4807.038", "N", "01131.000", "E", "1", "08",
                                 "0.9", "545.4", "M", "46.9", "M", "", ""}));
    CHECK(splits_into(rmc_body, {"GPRMC", "123519", "A", "4807.038", "N", "01131.000", "E",
                                 "022.4", "084.4", "230394", "003.1", "W"}));
}

void test_field_count_is_commas_plus_one() {
    for (std::size_t count = 1; count <= kMaxFields; ++count) {
        const std::optional<NmeaFields> fields = split_fields(repeated_fields(count));
        CHECK(fields.has_value() && fields->size() == count);
    }
}

void test_too_many_fields() {
    CHECK(!split_fields(repeated_fields(kMaxFields + 1)).has_value());
    CHECK(!split_fields(std::string(kMaxFields, ',')).has_value());
    CHECK(split_fields(std::string(kMaxFields - 1, ',')).has_value());
    CHECK(!split_fields(std::string(max_body_size, ',')).has_value());
}

void test_field_out_of_range() {
    const NmeaFields empty{};
    CHECK(empty.size() == 0);
    CHECK(empty.field(0).empty());

    const std::optional<NmeaFields> fields = split_fields("A,B");
    CHECK(fields.has_value());
    if (fields.has_value()) {
        CHECK(fields->field(1) == "B");
        CHECK(fields->field(2).empty());
        CHECK(fields->field(kMaxFields).empty());
        CHECK(fields->field(static_cast<std::size_t>(-1)).empty());
    }
}

void test_push_capacity() {
    NmeaFields fields;
    for (std::size_t index = 0; index < kMaxFields; ++index) {
        CHECK(fields.push(index + 1 == kMaxFields ? "last" : "X"));
    }
    CHECK(!fields.push("extra"));
    CHECK(fields.size() == kMaxFields);
    CHECK(fields.field(kMaxFields - 1) == "last");
}

void test_fields_point_into_sentence() {
    constexpr std::string_view sentence = "AB,,C";
    const std::optional<NmeaFields> fields = split_fields(sentence);
    CHECK(fields.has_value());
    if (fields.has_value()) {
        CHECK(fields->field(0).data() == sentence.data());
        CHECK(fields->field(1).data() == sentence.data() + 3);
        CHECK(fields->field(2).data() == sentence.data() + 4);
    }
}

void test_framer_output_into_tokeniser() {
    NmeaParser parser;
    bool completed = false;
    for (const char byte : gga) {
        completed = parser.receive_byte(static_cast<unsigned char>(byte));
    }
    CHECK(completed);

    const std::optional<NmeaFields> fields = split_fields(parser.get_sentence());
    CHECK(fields.has_value());
    if (fields.has_value()) {
        CHECK(fields->size() == 15);
        CHECK(fields->field(0) == "GPGGA");
        CHECK(fields->field(2) == "4807.038");
        CHECK(fields->field(3) == "N");
        CHECK(fields->field(6) == "1");
        CHECK(fields->field(14).empty());
    }
}

}

int main() {
    test_initial_state();
    test_valid_sentences();
    test_line_endings();
    test_hex_case();
    test_empty_body_and_fields();
    test_true_only_on_last_byte();
    test_checksum_mismatch();
    test_single_bit_corruption();
    test_transposition_is_not_detected();
    test_malformed_checksum();
    test_missing_checksum();
    test_junk_after_checksum();
    test_body_byte_range();
    test_bytes_outside_sentences();
    test_dollar_restarts_sentence();
    test_length_limit();
    test_stream_of_sentences();
    test_sentence_survives_until_next_dollar();

    test_split_plain_fields();
    test_split_empty_fields();
    test_split_keeps_content_as_is();
    test_split_real_sentences();
    test_field_count_is_commas_plus_one();
    test_too_many_fields();
    test_field_out_of_range();
    test_push_capacity();
    test_fields_point_into_sentence();
    test_framer_output_into_tokeniser();

    if (g_failures != 0) {
        std::cerr << g_failures << " checks failed\n";
        return 1;
    }

    std::cout << "All tests passed" << '\n';

    return 0;
}
