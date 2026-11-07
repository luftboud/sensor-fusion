#include <cstddef>
#include <cstdint>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "sensor_fusion/io/nmea/nmea_framer.h"
#include "sensor_fusion/io/nmea/nmea_tokeniser.h"

namespace {

using sensor_fusion::kMaxFields;
using sensor_fusion::NmeaFields;
using sensor_fusion::NmeaParser;
using sensor_fusion::SplitFields;

int failure_count = 0;

void Check(bool ok, const char* text, int line) {
  if (!ok) {
    std::cerr << "line " << line << ": " << text << '\n';
    ++failure_count;
  }
}

#define CHECK(condition) \
  Check(static_cast<bool>(condition), #condition, __LINE__)

using Sentences = std::vector<std::string>;

constexpr std::string_view kGgaBody =
    "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
constexpr std::string_view kGga =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";
constexpr std::string_view kRmcBody =
    "GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W";
constexpr std::string_view kRmc =
    "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A\r\n";

constexpr std::size_t kMaxBodySize = NmeaParser::kMaxBodySize;

// Sentences are copied on the byte that completes them, because sentence()
// is only a view of the parser's buffer.
Sentences Collect(NmeaParser& parser, std::string_view input) {
  Sentences sentences;
  for (const char byte : input) {
    if (parser.ReceiveByte(static_cast<unsigned char>(byte))) {
      sentences.emplace_back(parser.sentence());
    }
  }
  return sentences;
}

Sentences Collect(std::string_view input) {
  NmeaParser parser;
  return Collect(parser, input);
}

bool Accepts(std::string_view input, std::string_view expected) {
  const Sentences sentences = Collect(input);
  return sentences.size() == 1 && sentences.front() == expected;
}

// A rejected input yields nothing and must not break the sentence after it.
bool Rejects(std::string_view input) {
  NmeaParser parser;
  return Collect(parser, input).empty() &&
         Collect(parser, "$AB*03\r\n") == Sentences{"AB"};
}

// Builds a sentence with a correct checksum, so that a test can reject it for
// another reason. The literal sentences above keep this helper honest.
std::string Frame(std::string_view body) {
  constexpr std::string_view kDigits = "0123456789ABCDEF";

  std::uint8_t checksum = 0;
  for (const char byte : body) {
    checksum ^= static_cast<std::uint8_t>(byte);
  }

  std::string sentence = "$";
  sentence += body;
  sentence += '*';
  sentence += kDigits[checksum / 16];
  sentence += kDigits[checksum % 16];
  sentence += "\r\n";
  return sentence;
}

void TestInitialState() {
  const NmeaParser parser;
  CHECK(parser.sentence().empty());
}

void TestValidSentences() {
  CHECK(Accepts("$AB*03\r\n", "AB"));
  CHECK(Accepts(kGga, kGgaBody));
  CHECK(Accepts(kRmc, kRmcBody));
}

void TestLineEndings() {
  CHECK(Accepts("$AB*03\n", "AB"));
  CHECK(Accepts("$AB*03\r\n", "AB"));
  CHECK(Accepts("$AB*03\r\r\n", "AB"));
  CHECK(Collect("$AB*03").empty());
  CHECK(Collect("$AB*03\r").empty());
}

void TestHexCase() {
  CHECK(Accepts("$AJ*0B\r\n", "AJ"));
  CHECK(Accepts("$AJ*0b\r\n", "AJ"));
  CHECK(Accepts("$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,"
                "003.1,W*6a\r\n",
                kRmcBody));
}

void TestEmptyBodyAndFields() {
  CHECK(Accepts("$*00\r\n", ""));
  CHECK(Accepts("$,,,*2C\r\n", ",,,"));
  CHECK(Accepts(Frame("GPGGA,,,,,,0,,,,,,,,"), "GPGGA,,,,,,0,,,,,,,,"));
}

void TestTrueOnlyOnLastByte() {
  NmeaParser parser;
  for (std::size_t index = 0; index + 1 < kGga.size(); ++index) {
    CHECK(!parser.ReceiveByte(static_cast<unsigned char>(kGga[index])));
  }
  CHECK(parser.ReceiveByte(static_cast<unsigned char>(kGga.back())));
  CHECK(parser.sentence() == kGgaBody);
}

void TestChecksumMismatch() {
  CHECK(Rejects("$AB*04\r\n"));
  CHECK(Rejects("$AB*13\r\n"));
  CHECK(Rejects("$AB*30\r\n"));
  CHECK(Rejects("$AB*00\r\n"));
  CHECK(Rejects("$AB*FF\r\n"));
  CHECK(Rejects("$*01\r\n"));
  CHECK(Rejects("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,"
                ",*48\r\n"));
}

void TestSingleBitCorruption() {
  for (std::size_t index = 0; index < kGga.size(); ++index) {
    for (int bit = 0; bit < 8; ++bit) {
      std::string corrupted(kGga);
      const int flipped = static_cast<unsigned char>(kGga[index]) ^ (1 << bit);
      corrupted[index] = static_cast<char>(flipped);
      CHECK(Rejects(corrupted));
    }
  }
}

void TestTranspositionIsNotDetected() { CHECK(Accepts("$BA*03\r\n", "BA")); }

void TestMalformedChecksum() {
  CHECK(Rejects("$AB*0G\r\n"));
  CHECK(Rejects("$AB*G3\r\n"));
  CHECK(Rejects("$AB*\r\n"));
  CHECK(Rejects("$AB*3\r\n"));
  CHECK(Rejects("$AB*033\r\n"));
  CHECK(Rejects("$AB**03\r\n"));
  CHECK(Rejects("$AB*0 3\r\n"));
}

void TestMissingChecksum() {
  CHECK(Rejects("$AB\r\n"));
  CHECK(Rejects("$AB\n"));
  CHECK(Rejects("$AB\r*03\r\n"));
  CHECK(Rejects("$AB\n*03\r\n"));
}

void TestJunkAfterChecksum() {
  CHECK(Rejects("$AB*03X\r\n"));
  CHECK(Rejects("$AB*03 \r\n"));
  CHECK(Rejects("$AB*03\rX\n"));
}

void TestBodyByteRange() {
  for (int value = 0; value <= 0xFF; ++value) {
    if (value == '$' || value == '*') {
      continue;
    }

    std::string body = "A";
    body += static_cast<char>(value);
    body += 'B';

    const bool printable = value >= 0x20 && value <= 0x7E;
    CHECK(printable ? Accepts(Frame(body), body) : Rejects(Frame(body)));
  }
}

void TestBytesOutsideSentences() {
  CHECK(Accepts("garbage$AB*03\r\n", "AB"));
  CHECK(Accepts("\r\n\r\n$AB*03\r\n", "AB"));
  CHECK(Accepts("\xFF\x80*03\r\n$AB*03\r\n", "AB"));
  CHECK(Rejects("AB*03\r\n"));
  CHECK(Rejects("*03\r\n"));
  CHECK(Rejects("\r\n"));
}

void TestDollarRestartsSentence() {
  CHECK(Accepts("$$AB*03\r\n", "AB"));
  CHECK(Accepts("$XY$AB*03\r\n", "AB"));
  CHECK(Accepts("$XY*$AB*03\r\n", "AB"));
  CHECK(Accepts("$XY*0$AB*03\r\n", "AB"));
  CHECK(Accepts("$XY*03$AB*03\r\n", "AB"));
  CHECK(Accepts("$XY*03\r$AB*03\r\n", "AB"));
}

void TestLengthLimit() {
  const std::string longest(kMaxBodySize, 'A');
  const std::string too_long(kMaxBodySize + 1, 'A');
  const std::string far_too_long(kMaxBodySize * 4, 'A');

  CHECK(Accepts(Frame(longest), longest));
  CHECK(Rejects(Frame(too_long)));
  CHECK(Rejects(Frame(far_too_long)));
}

void TestStreamOfSentences() {
  std::string stream = "noise";
  stream += kGga;
  stream += "$AB*04\r\n";
  stream += "$AB*03\r\n";
  stream += "$AB*03\n";
  stream += "$AB";
  stream += kRmc;

  const Sentences expected = {std::string(kGgaBody), "AB", "AB",
                              std::string(kRmcBody)};
  CHECK(Collect(stream) == expected);
}

void TestSentenceSurvivesUntilNextDollar() {
  NmeaParser parser;
  CHECK(Collect(parser, "$AB*03\r\n") == Sentences{"AB"});
  CHECK(parser.sentence() == "AB");

  CHECK(Collect(parser, "noise*\r\n").empty());
  CHECK(parser.sentence() == "AB");

  CHECK(Collect(parser, "$").empty());
  CHECK(parser.sentence().empty());
}

bool SplitsInto(std::string_view sentence,
                const std::vector<std::string_view>& expected,
                char delimiter = ',') {
  const std::optional<NmeaFields> fields = SplitFields(sentence, delimiter);
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
std::string RepeatedFields(std::size_t count) {
  std::string sentence = "X";
  for (std::size_t index = 1; index < count; ++index) {
    sentence += ",X";
  }
  return sentence;
}

void TestSplitPlainFields() {
  CHECK(SplitsInto("A", {"A"}));
  CHECK(SplitsInto("A,B", {"A", "B"}));
  CHECK(SplitsInto("GGA,1,2.5,N", {"GGA", "1", "2.5", "N"}));
}

void TestSplitEmptyFields() {
  CHECK(SplitsInto("", {""}));
  CHECK(SplitsInto(",", {"", ""}));
  CHECK(SplitsInto(",,,", {"", "", "", ""}));
  CHECK(SplitsInto(",A", {"", "A"}));
  CHECK(SplitsInto("A,", {"A", ""}));
  CHECK(SplitsInto("A,,B", {"A", "", "B"}));
}

void TestSplitKeepsContentAsIs() {
  CHECK(SplitsInto(" A , B ", {" A ", " B "}));
  CHECK(SplitsInto("AB*03", {"AB*03"}));
  CHECK(SplitsInto("$GP,GGA\r\n", {"$GP", "GGA\r\n"}));
}

void TestSplitRealSentences() {
  CHECK(SplitsInto(kGgaBody, {"GPGGA", "123519", "4807.038", "N", "01131.000",
                              "E", "1", "08", "0.9", "545.4", "M", "46.9", "M",
                              "", ""}));
  CHECK(SplitsInto(kRmcBody, {"GPRMC", "123519", "A", "4807.038", "N",
                              "01131.000", "E", "022.4", "084.4", "230394",
                              "003.1", "W"}));
}

void TestSplitSpaceDelimiter() {
  CHECK(SplitsInto("A", {"A"}, ' '));
  CHECK(SplitsInto("A B C", {"A", "B", "C"}, ' '));
  CHECK(SplitsInto("", {""}, ' '));
  CHECK(SplitsInto(" ", {"", ""}, ' '));
  CHECK(SplitsInto("A  B", {"A", "", "B"}, ' '));
  CHECK(SplitsInto(" A", {"", "A"}, ' '));
  CHECK(SplitsInto("A ", {"A", ""}, ' '));
}

void TestSplitSpaceDelimiterIgnoresOtherSeparators() {
  CHECK(SplitsInto("1,5 2", {"1,5", "2"}, ' '));
  CHECK(SplitsInto("A\tB C", {"A\tB", "C"}, ' '));
  CHECK(SplitsInto("1 2\r\n", {"1", "2\r\n"}, ' '));
  CHECK(SplitsInto("A,B C", {"A", "B C"}, ','));
  CHECK(SplitsInto("A,B C", {"A", "B C"}));
}

void TestSplitKittiOxtsLine() {
  // Shaped like a KITTI oxts record: 30 space-separated values. The numbers
  // are illustrative, not taken from a real drive.
  constexpr std::string_view kLine =
      "49.015003823272 8.4342971002335 116.43032836914 0.035752 0.00903 "
      "-2.6087069803847 -4.1729700268209 2.4888127772796 0.12542351318744 "
      "4.8216164543621 0.081180209286318 0.26213843786713 0.2068395575419 "
      "9.7920389437861 -0.35232463246227 0.67116108941043 9.7487722519064 "
      "-0.0070934627317466 -0.0034766233853478 0.030541962571628 "
      "-0.0057622734932009 -0.0049123227213927 0.030683538040474 "
      "0.21650295197093 0.12401215202203 4 10 4 4 0";

  const std::optional<NmeaFields> fields = SplitFields(kLine, ' ');
  CHECK(fields.has_value());
  if (fields.has_value()) {
    CHECK(fields->size() == 30);
    CHECK(fields->field(0) == "49.015003823272");
    CHECK(fields->field(5) == "-2.6087069803847");
    CHECK(fields->field(26) == "10");
    CHECK(fields->field(29) == "0");
    CHECK(fields->field(30).empty());
  }

  CHECK(SplitsInto(kLine, {kLine}));
}

void TestFieldCountIsCommasPlusOne() {
  for (std::size_t count = 1; count <= kMaxFields; ++count) {
    const std::optional<NmeaFields> fields = SplitFields(RepeatedFields(count));
    CHECK(fields.has_value() && fields->size() == count);
  }
}

void TestTooManyFields() {
  CHECK(!SplitFields(RepeatedFields(kMaxFields + 1)).has_value());
  CHECK(!SplitFields(std::string(kMaxFields, ',')).has_value());
  CHECK(SplitFields(std::string(kMaxFields - 1, ',')).has_value());
  CHECK(!SplitFields(std::string(kMaxBodySize, ',')).has_value());
}

void TestFieldOutOfRange() {
  const NmeaFields empty;
  CHECK(empty.size() == 0);
  CHECK(empty.field(0).empty());

  const std::optional<NmeaFields> fields = SplitFields("A,B");
  CHECK(fields.has_value());
  if (fields.has_value()) {
    CHECK(fields->field(1) == "B");
    CHECK(fields->field(2).empty());
    CHECK(fields->field(kMaxFields).empty());
    CHECK(fields->field(static_cast<std::size_t>(-1)).empty());
  }
}

void TestPushCapacity() {
  NmeaFields fields;
  for (std::size_t index = 0; index < kMaxFields; ++index) {
    CHECK(fields.Push(index + 1 == kMaxFields ? "last" : "X"));
  }
  CHECK(!fields.Push("extra"));
  CHECK(fields.size() == kMaxFields);
  CHECK(fields.field(kMaxFields - 1) == "last");
}

void TestFieldsPointIntoSentence() {
  constexpr std::string_view kSentence = "AB,,C";
  const std::optional<NmeaFields> fields = SplitFields(kSentence);
  CHECK(fields.has_value());
  if (fields.has_value()) {
    CHECK(fields->field(0).data() == kSentence.data());
    CHECK(fields->field(1).data() == kSentence.data() + 3);
    CHECK(fields->field(2).data() == kSentence.data() + 4);
  }
}

void TestFramerOutputIntoTokeniser() {
  NmeaParser parser;
  bool completed = false;
  for (const char byte : kGga) {
    completed = parser.ReceiveByte(static_cast<unsigned char>(byte));
  }
  CHECK(completed);

  const std::optional<NmeaFields> fields = SplitFields(parser.sentence());
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

}  // namespace

int main() {
  TestInitialState();
  TestValidSentences();
  TestLineEndings();
  TestHexCase();
  TestEmptyBodyAndFields();
  TestTrueOnlyOnLastByte();
  TestChecksumMismatch();
  TestSingleBitCorruption();
  TestTranspositionIsNotDetected();
  TestMalformedChecksum();
  TestMissingChecksum();
  TestJunkAfterChecksum();
  TestBodyByteRange();
  TestBytesOutsideSentences();
  TestDollarRestartsSentence();
  TestLengthLimit();
  TestStreamOfSentences();
  TestSentenceSurvivesUntilNextDollar();

  TestSplitPlainFields();
  TestSplitEmptyFields();
  TestSplitKeepsContentAsIs();
  TestSplitRealSentences();
  TestSplitSpaceDelimiter();
  TestSplitSpaceDelimiterIgnoresOtherSeparators();
  TestSplitKittiOxtsLine();
  TestFieldCountIsCommasPlusOne();
  TestTooManyFields();
  TestFieldOutOfRange();
  TestPushCapacity();
  TestFieldsPointIntoSentence();
  TestFramerOutputIntoTokeniser();

  if (failure_count != 0) {
    std::cerr << failure_count << " checks failed\n";
    return 1;
  }

  std::cout << "All tests passed\n";
  return 0;
}
