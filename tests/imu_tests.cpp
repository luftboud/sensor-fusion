#include <sys/stat.h>
#include <unistd.h>

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "sensor_fusion/io/imu/imu.h"

namespace {

using sensor_fusion::ImuSample;
using sensor_fusion::KittiError;
using sensor_fusion::KittiReadResult;
using sensor_fusion::OxtsRecord;
using sensor_fusion::ParseKittiTimestamp;
using sensor_fusion::ParseOxtsLine;
using sensor_fusion::ReadKittiOxts;
using sensor_fusion::ToImuSample;
using sensor_fusion::ToString;
using sensor_fusion::internal::DataFileName;
using sensor_fusion::internal::StripTrailingWhitespace;

int failure_count = 0;

void Check(bool ok, const char* text, int line) {
  if (!ok) {
    std::cerr << "line " << line << ": " << text << '\n';
    ++failure_count;
  }
}

#define CHECK(condition) \
  Check(static_cast<bool>(condition), #condition, __LINE__)

bool Near(double actual, double expected) {
  return std::abs(actual - expected) < 1e-9;
}

// First line of 2011_09_26_drive_0009_sync/oxts/data/0000000000.txt.
constexpr std::string_view kRealLine =
    "49.009347760599 8.4371344308375 114.45652008057 0.072242 0.001566 "
    "-0.2697266732051 -2.9301818443449 10.286884437741 10.695750797331 "
    "0.082893715568935 -0.075015404010604 0.0059525410318346 0.845164147132 "
    "9.9149729634473 0.041978129261809 0.13652761309179 9.9501668070309 "
    "0.016662646247367 -0.036831580656409 0.003311640516573 "
    "0.016662387052679 -0.036976147506788 0.00060422104178454 "
    "0.42856154750514 0.057982756057297 4 7 4 4 0";

// "1 2 3 ... 30": field i holds the value i + 1, which exposes any mix-up in
// the field order.
std::string NumberedLine() {
  std::string line;
  for (int value = 1; value <= 30; ++value) {
    if (value > 1) {
      line += ' ';
    }
    line += std::to_string(value);
  }
  return line;
}

// NumberedLine() with field `index` replaced by `text`.
std::string NumberedLineWith(std::size_t index, std::string_view text) {
  std::string line;
  for (std::size_t i = 0; i < 30; ++i) {
    if (i > 0) {
      line += ' ';
    }
    line += i == index ? std::string(text) : std::to_string(i + 1);
  }
  return line;
}

bool ParsesOxts(std::string_view line) {
  return ParseOxtsLine(line).has_value();
}

// A temporary oxts directory with timestamps.txt and data/. It is removed
// when the object goes out of scope.
class OxtsDir {
 public:
  OxtsDir() {
    const char* tmp = std::getenv("TMPDIR");
    std::string pattern = (tmp != nullptr && *tmp != '\0') ? tmp : "/tmp";
    pattern += "/imu_tests_XXXXXX";
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    if (mkdtemp(buffer.data()) != nullptr) {
      path_ = buffer.data();
      mkdir((path_ + "/data").c_str(), 0700);
    }
  }

  OxtsDir(const OxtsDir&) = delete;
  OxtsDir& operator=(const OxtsDir&) = delete;

  ~OxtsDir() {
    for (const std::string& file : files_) {
      unlink(file.c_str());
    }
    rmdir((path_ + "/data").c_str());
    rmdir(path_.c_str());
  }

  [[nodiscard]] const std::string& path() const { return path_; }

  void WriteTimestamps(std::string_view content) {
    Write("timestamps.txt", content);
  }

  void WriteData(std::size_t index, std::string_view content) {
    Write("data/" + DataFileName(index), content);
  }

 private:
  void Write(const std::string& relative_path, std::string_view content) {
    if (path_.empty()) {
      return;  // mkdtemp() failed; never write relative to "/".
    }
    const std::string full_path = path_ + "/" + relative_path;
    std::ofstream(full_path, std::ios::binary) << content;
    files_.push_back(full_path);
  }

  std::string path_;
  std::vector<std::string> files_;
};

bool FailsWith(const KittiReadResult& result, KittiError error,
               std::size_t index) {
  return !result.ok() && result.error == error && result.index == index &&
         result.samples.empty();
}

void TestStripTrailingWhitespace() {
  CHECK(StripTrailingWhitespace("abc") == "abc");
  CHECK(StripTrailingWhitespace("abc \t\r\n") == "abc");
  CHECK(StripTrailingWhitespace("  abc") == "  abc");
  CHECK(StripTrailingWhitespace(" \t\r\n").empty());
  CHECK(StripTrailingWhitespace("").empty());
}

void TestParseRealOxtsLine() {
  const std::optional<OxtsRecord> record = ParseOxtsLine(kRealLine);
  CHECK(record.has_value());
  if (!record.has_value()) {
    return;
  }
  CHECK(record->lat == 49.009347760599);
  CHECK(record->lon == 8.4371344308375);
  CHECK(record->alt == 114.45652008057);
  CHECK(record->yaw == -0.2697266732051);
  CHECK(record->az == 9.9149729634473);
  CHECK(record->wz == 0.003311640516573);
  CHECK(record->pos_accuracy == 0.42856154750514);
  CHECK(record->vel_accuracy == 0.057982756057297);
  CHECK(record->navstat == 4);
  CHECK(record->numsats == 7);
  CHECK(record->posmode == 4);
  CHECK(record->velmode == 4);
  CHECK(record->orimode == 0);
}

void TestParseFieldOrder() {
  const std::optional<OxtsRecord> r = ParseOxtsLine(NumberedLine());
  CHECK(r.has_value());
  if (!r.has_value()) {
    return;
  }
  CHECK(r->lat == 1 && r->lon == 2 && r->alt == 3);
  CHECK(r->roll == 4 && r->pitch == 5 && r->yaw == 6);
  CHECK(r->vn == 7 && r->ve == 8);
  CHECK(r->vf == 9 && r->vl == 10 && r->vu == 11);
  CHECK(r->ax == 12 && r->ay == 13 && r->az == 14);
  CHECK(r->af == 15 && r->al == 16 && r->au == 17);
  CHECK(r->wx == 18 && r->wy == 19 && r->wz == 20);
  CHECK(r->wf == 21 && r->wl == 22 && r->wu == 23);
  CHECK(r->pos_accuracy == 24 && r->vel_accuracy == 25);
  CHECK(r->navstat == 26 && r->numsats == 27 && r->posmode == 28);
  CHECK(r->velmode == 29 && r->orimode == 30);
}

// Some KITTI files write the status fields as "4.00000000000000".
void TestParseKittiWholeNumberForms() {
  const std::string line =
      NumberedLine().substr(0, NumberedLine().find(" 26")) +
      " 4.00000000000000 10.00000000000000 -1.00000000000000 -1 0";
  const std::optional<OxtsRecord> record = ParseOxtsLine(line);
  CHECK(record.has_value());
  if (record.has_value()) {
    CHECK(record->navstat == 4);
    CHECK(record->numsats == 10);
    CHECK(record->posmode == -1);
    CHECK(record->velmode == -1);
    CHECK(record->orimode == 0);
  }
  CHECK(ParsesOxts(NumberedLineWith(25, "2147483647")));
  CHECK(ParsesOxts(NumberedLineWith(25, "-2147483648")));
}

void TestParseTrailingWhitespace() {
  const std::string line(kRealLine);
  CHECK(ParsesOxts(line + "\n"));
  CHECK(ParsesOxts(line + "\r\n"));
  CHECK(ParsesOxts(line + " "));
  CHECK(ParsesOxts(line + " \t\r\n"));
}

void TestParseRejectsWrongFieldCount() {
  const std::string line = NumberedLine();
  CHECK(!ParsesOxts(line.substr(0, line.rfind(' '))));
  CHECK(!ParsesOxts(line + " 31"));
  CHECK(!ParsesOxts(""));
  CHECK(!ParsesOxts("   "));
  CHECK(!ParsesOxts(" " + line));
  CHECK(!ParsesOxts(NumberedLineWith(10, "")));
  CHECK(!ParsesOxts(line + ",31"));
}

void TestParseRejectsBadNumbers() {
  for (const std::string_view bad :
       {"abc", "1.5x", "nan", "inf", "-inf", "+1", "0x10", "1,5", "--1"}) {
    CHECK(!ParsesOxts(NumberedLineWith(0, bad)));
    CHECK(!ParsesOxts(NumberedLineWith(24, bad)));
    CHECK(!ParsesOxts(NumberedLineWith(29, bad)));
  }
}

void TestParseRejectsBadStatusCodes() {
  for (const std::string_view bad :
       {"4.5", "0.000001", "1e10", "2147483648", "-2147483649", "nan"}) {
    CHECK(!ParsesOxts(NumberedLineWith(25, bad)));
    CHECK(!ParsesOxts(NumberedLineWith(29, bad)));
  }
  CHECK(ParsesOxts(NumberedLineWith(24, "4.5")));
}

void TestToImuSample() {
  const std::optional<OxtsRecord> record = ParseOxtsLine(NumberedLine());
  CHECK(record.has_value());
  if (!record.has_value()) {
    return;
  }
  const ImuSample sample = ToImuSample(1.25, *record);
  CHECK(sample.t == 1.25);
  CHECK(sample.accel[0] == 12 && sample.accel[1] == 13 &&
        sample.accel[2] == 14);
  CHECK(sample.gyro[0] == 18 && sample.gyro[1] == 19 && sample.gyro[2] == 20);
}

void TestTimestampValues() {
  CHECK(ParseKittiTimestamp("1970-01-01 00:00:00.0") == 0);
  CHECK(ParseKittiTimestamp("1970-01-01 00:00:01.000000001") == 1000000001);
  CHECK(ParseKittiTimestamp("2011-09-26 13:08:24.969505176") ==
        1317042504969505176);
  CHECK(ParseKittiTimestamp("2012-02-29 00:00:00.0") == 1330473600000000000);
}

void TestTimestampFraction() {
  CHECK(ParseKittiTimestamp("1970-01-01 00:00:00.5") == 500000000);
  CHECK(ParseKittiTimestamp("1970-01-01 00:00:00.05") == 50000000);
  CHECK(ParseKittiTimestamp("1970-01-01 00:00:00.123456789") == 123456789);
  CHECK(!ParseKittiTimestamp("1970-01-01 00:00:00.1234567890"));
  CHECK(!ParseKittiTimestamp("1970-01-01 00:00:00."));
  CHECK(!ParseKittiTimestamp("1970-01-01 00:00:00"));
  CHECK(!ParseKittiTimestamp("1970-01-01 00:00:00.5x"));
}

void TestTimestampCalendar() {
  CHECK(ParseKittiTimestamp("2000-02-29 00:00:00.0").has_value());
  CHECK(!ParseKittiTimestamp("2011-02-29 00:00:00.0"));
  CHECK(!ParseKittiTimestamp("1900-02-29 00:00:00.0"));
  CHECK(!ParseKittiTimestamp("2011-04-31 00:00:00.0"));
  CHECK(!ParseKittiTimestamp("2011-00-10 00:00:00.0"));
  CHECK(!ParseKittiTimestamp("2011-13-10 00:00:00.0"));
  CHECK(!ParseKittiTimestamp("2011-09-00 00:00:00.0"));
}

void TestTimestampTimeOfDay() {
  CHECK(ParseKittiTimestamp("2011-09-26 23:59:59.999999999").has_value());
  CHECK(!ParseKittiTimestamp("2011-09-26 24:00:00.0"));
  CHECK(!ParseKittiTimestamp("2011-09-26 12:60:00.0"));
  CHECK(!ParseKittiTimestamp("2011-09-26 12:00:60.0"));
}

void TestTimestampFormat() {
  CHECK(ParseKittiTimestamp("2011-09-26 13:08:24.5\r\n") ==
        ParseKittiTimestamp("2011-09-26 13:08:24.5"));
  CHECK(!ParseKittiTimestamp(""));
  CHECK(!ParseKittiTimestamp(" 2011-09-26 13:08:24.5"));
  CHECK(!ParseKittiTimestamp("2011/09/26 13:08:24.5"));
  CHECK(!ParseKittiTimestamp("2011-09-26T13:08:24.5"));
  CHECK(!ParseKittiTimestamp("2011-09-26 13-08-24.5"));
  CHECK(!ParseKittiTimestamp("2011-09-26 1:08:24.55"));
  CHECK(!ParseKittiTimestamp("2011-+9-26 13:08:24.5"));
  CHECK(!ParseKittiTimestamp("-011-09-26 13:08:24.5"));
  CHECK(!ParseKittiTimestamp("2011-09-26 -1:08:24.5"));
  CHECK(!ParseKittiTimestamp("2011-09-26 13:08:24.-5"));
  CHECK(!ParseKittiTimestamp("2011-09-26 13:08:24,5"));
}

void TestDataFileName() {
  CHECK(DataFileName(0) == "0000000000.txt");
  CHECK(DataFileName(42) == "0000000042.txt");
  CHECK(DataFileName(9999999999) == "9999999999.txt");
  CHECK(DataFileName(12345678901) == "12345678901.txt");
}

void TestErrorNames() {
  const std::vector<KittiError> errors = {
      KittiError::kNone,           KittiError::kNoTimestampsFile,
      KittiError::kNoTimestamps,   KittiError::kBadTimestamp,
      KittiError::kTimeNotIncreasing, KittiError::kMissingDataFile,
      KittiError::kBadDataLine,    KittiError::kExtraDataFile,
  };
  std::set<std::string_view> names;
  for (const KittiError error : errors) {
    CHECK(!ToString(error).empty());
    names.insert(ToString(error));
  }
  CHECK(names.size() == errors.size());
  CHECK(ToString(KittiError::kNone) == "no error");
}

void TestReadValidDirectory() {
  OxtsDir dir;
  dir.WriteTimestamps(
      "2011-09-26 13:08:24.0\n"
      "2011-09-26 13:08:24.1\n"
      "2011-09-26 13:08:24.25\n");
  dir.WriteData(0, std::string(kRealLine) + "\n");
  dir.WriteData(1, NumberedLine() + "\n");
  dir.WriteData(2, NumberedLineWith(11, "-3.5") + "\n");

  const KittiReadResult result = ReadKittiOxts(dir.path());
  CHECK(result.ok());
  CHECK(result.error == KittiError::kNone);
  CHECK(result.start_ns == 1317042504000000000);
  CHECK(result.samples.size() == 3);
  if (result.samples.size() == 3) {
    CHECK(result.samples[0].t == 0.0);
    CHECK(Near(result.samples[1].t, 0.1));
    CHECK(Near(result.samples[2].t, 0.25));
    CHECK(result.samples[0].accel[2] == 9.9149729634473);
    CHECK(result.samples[0].gyro[2] == 0.003311640516573);
    CHECK(result.samples[1].accel[0] == 12);
    CHECK(result.samples[2].accel[0] == -3.5);
  }

  CHECK(ReadKittiOxts(dir.path() + "/").samples.size() == 3);
}

void TestReadCrlfAndTrailingBlankLines() {
  OxtsDir dir;
  dir.WriteTimestamps(
      "2011-09-26 13:08:24.0\r\n"
      "2011-09-26 13:08:24.1\r\n"
      "\r\n"
      "\n");
  dir.WriteData(0, NumberedLine() + "\r\n");
  dir.WriteData(1, NumberedLine() + "\r\nignored second line\r\n");

  const KittiReadResult result = ReadKittiOxts(dir.path());
  CHECK(result.ok());
  CHECK(result.samples.size() == 2);
}

void TestReadMissingTimestamps() {
  OxtsDir dir;
  CHECK(FailsWith(ReadKittiOxts(dir.path()), KittiError::kNoTimestampsFile,
                  0));
  CHECK(FailsWith(ReadKittiOxts(dir.path() + "/does_not_exist"),
                  KittiError::kNoTimestampsFile, 0));
}

void TestReadEmptyTimestamps() {
  OxtsDir empty;
  empty.WriteTimestamps("");
  CHECK(FailsWith(ReadKittiOxts(empty.path()), KittiError::kNoTimestamps, 0));

  OxtsDir blank;
  blank.WriteTimestamps("\n \r\n\n");
  CHECK(FailsWith(ReadKittiOxts(blank.path()), KittiError::kNoTimestamps, 0));
}

void TestReadBadTimestamp() {
  OxtsDir dir;
  dir.WriteTimestamps(
      "2011-09-26 13:08:24.0\n"
      "2011-09-26 13:08:24\n");
  dir.WriteData(0, NumberedLine());
  dir.WriteData(1, NumberedLine());
  CHECK(FailsWith(ReadKittiOxts(dir.path()), KittiError::kBadTimestamp, 1));

  OxtsDir gap;
  gap.WriteTimestamps(
      "2011-09-26 13:08:24.0\n"
      "\n"
      "2011-09-26 13:08:24.2\n");
  gap.WriteData(0, NumberedLine());
  CHECK(FailsWith(ReadKittiOxts(gap.path()), KittiError::kBadTimestamp, 1));
}

void TestReadTimeNotIncreasing() {
  OxtsDir equal;
  equal.WriteTimestamps(
      "2011-09-26 13:08:24.1\n"
      "2011-09-26 13:08:24.1\n");
  equal.WriteData(0, NumberedLine());
  equal.WriteData(1, NumberedLine());
  CHECK(FailsWith(ReadKittiOxts(equal.path()), KittiError::kTimeNotIncreasing,
                  1));

  OxtsDir backwards;
  backwards.WriteTimestamps(
      "2011-09-26 13:08:24.1\n"
      "2011-09-26 13:08:24.2\n"
      "2011-09-26 13:08:24.0\n");
  for (std::size_t i = 0; i < 3; ++i) {
    backwards.WriteData(i, NumberedLine());
  }
  CHECK(FailsWith(ReadKittiOxts(backwards.path()),
                  KittiError::kTimeNotIncreasing, 2));
}

void TestReadMissingDataFile() {
  OxtsDir dir;
  dir.WriteTimestamps(
      "2011-09-26 13:08:24.0\n"
      "2011-09-26 13:08:24.1\n"
      "2011-09-26 13:08:24.2\n");
  dir.WriteData(0, NumberedLine());
  dir.WriteData(2, NumberedLine());
  CHECK(FailsWith(ReadKittiOxts(dir.path()), KittiError::kMissingDataFile, 1));
}

void TestReadBadDataLine() {
  OxtsDir dir;
  dir.WriteTimestamps(
      "2011-09-26 13:08:24.0\n"
      "2011-09-26 13:08:24.1\n");
  dir.WriteData(0, NumberedLine());
  dir.WriteData(1, NumberedLineWith(26, "nan"));
  CHECK(FailsWith(ReadKittiOxts(dir.path()), KittiError::kBadDataLine, 1));

  OxtsDir empty_file;
  empty_file.WriteTimestamps("2011-09-26 13:08:24.0\n");
  empty_file.WriteData(0, "");
  CHECK(FailsWith(ReadKittiOxts(empty_file.path()), KittiError::kBadDataLine,
                  0));
}

void TestReadExtraDataFile() {
  OxtsDir dir;
  dir.WriteTimestamps(
      "2011-09-26 13:08:24.0\n"
      "2011-09-26 13:08:24.1\n");
  for (std::size_t i = 0; i < 3; ++i) {
    dir.WriteData(i, NumberedLine());
  }
  CHECK(FailsWith(ReadKittiOxts(dir.path()), KittiError::kExtraDataFile, 2));
}

}  // namespace

int main() {
  TestStripTrailingWhitespace();
  TestParseRealOxtsLine();
  TestParseFieldOrder();
  TestParseKittiWholeNumberForms();
  TestParseTrailingWhitespace();
  TestParseRejectsWrongFieldCount();
  TestParseRejectsBadNumbers();
  TestParseRejectsBadStatusCodes();
  TestToImuSample();

  TestTimestampValues();
  TestTimestampFraction();
  TestTimestampCalendar();
  TestTimestampTimeOfDay();
  TestTimestampFormat();

  TestDataFileName();
  TestErrorNames();

  TestReadValidDirectory();
  TestReadCrlfAndTrailingBlankLines();
  TestReadMissingTimestamps();
  TestReadEmptyTimestamps();
  TestReadBadTimestamp();
  TestReadTimeNotIncreasing();
  TestReadMissingDataFile();
  TestReadBadDataLine();
  TestReadExtraDataFile();

  if (failure_count != 0) {
    std::cerr << failure_count << " checks failed\n";
    return 1;
  }

  std::cout << "All tests passed\n";
  return 0;
}
