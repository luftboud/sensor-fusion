#include <unistd.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "sensor_fusion/ekf/scenario.h"
#include "sensor_fusion/io/gnss/gnss_csv.h"

namespace {

using sensor_fusion::GnssCsvError;
using sensor_fusion::GnssCsvResult;
using sensor_fusion::GnssFix;
using sensor_fusion::kGnssCsvHeader;
using sensor_fusion::ParseGnssCsv;
using sensor_fusion::ParseGnssCsvLine;
using sensor_fusion::ReadGnssCsv;
using sensor_fusion::ToString;

int failure_count = 0;

void Check(bool ok, const char* text, int line) {
  if (!ok) {
    std::cerr << "line " << line << ": " << text << '\n';
    ++failure_count;
  }
}

#define CHECK(condition) \
  Check(static_cast<bool>(condition), #condition, __LINE__)

// The first lines of docs/ekf-compare/gnss_full.csv from ref_ekf.py.
constexpr std::string_view kPythonCsv =
    "t,east,north\n"
    "5.000000000000000000e-01,6.060935039844732586e+00,"
    "-4.582129672942816256e+00\n"
    "1.500000000000000000e+00,1.765074825772709488e+01,"
    "-1.576112805849749776e+00\n"
    "2.500000000000000000e+00,1.944205912174712125e+01,"
    "-1.118620235908823268e+01\n";

GnssCsvResult Parse(std::string_view text) {
  std::istringstream input{std::string(text)};
  return ParseGnssCsv(input);
}

bool FailsWith(const GnssCsvResult& result, GnssCsvError error,
               std::size_t line) {
  return !result.ok() && result.error == error && result.line == line &&
         result.fixes.empty();
}

bool SameFix(const GnssFix& a, const GnssFix& b) {
  return a.t == b.t && a.east == b.east && a.north == b.north;
}

// A temporary file that is removed when the object goes out of scope. If
// mkstemp() fails, path() is empty and the destructor removes nothing.
class TempFile {
 public:
  explicit TempFile(std::string_view content) {
    const char* tmp = std::getenv("TMPDIR");
    std::string pattern = (tmp != nullptr && *tmp != '\0') ? tmp : "/tmp";
    pattern += "/gnss_csv_tests_XXXXXX";
    std::vector<char> buffer(pattern.begin(), pattern.end());
    buffer.push_back('\0');
    const int fd = mkstemp(buffer.data());
    if (fd < 0) {
      return;
    }
    close(fd);
    path_ = buffer.data();
    std::ofstream(path_, std::ios::binary) << content;
  }

  TempFile(const TempFile&) = delete;
  TempFile& operator=(const TempFile&) = delete;

  ~TempFile() {
    if (!path_.empty()) {
      unlink(path_.c_str());
    }
  }

  [[nodiscard]] const std::string& path() const { return path_; }

 private:
  std::string path_;
};

void TestParseLine() {
  const std::optional<GnssFix> fix = ParseGnssCsvLine(
      "5.000000000000000000e-01,6.060935039844732586e+00,"
      "-4.582129672942816256e+00");
  CHECK(fix.has_value());
  if (fix.has_value()) {
    CHECK(fix->t == 0.5);
    CHECK(fix->east == 6.060935039844732586e+00);
    CHECK(fix->north == -4.582129672942816256e+00);
  }

  // Other number forms that from_chars reads.
  const std::optional<GnssFix> plain = ParseGnssCsvLine("0.5,6,-4.25");
  CHECK(plain.has_value() &&
        SameFix(*plain, {.t = 0.5, .east = 6.0, .north = -4.25}));
  const std::optional<GnssFix> exponent = ParseGnssCsvLine("5e-1,1E3,-0");
  CHECK(exponent.has_value() &&
        SameFix(*exponent, {.t = 0.5, .east = 1000.0, .north = 0.0}));

  // Trailing spaces, tabs and CR.
  CHECK(ParseGnssCsvLine("1,2,3 \t\r").has_value());
}

void TestParseLineRejects() {
  const std::array<std::string_view, 17> bad = {
      "",          "1,2",       "1,2,3,4",   "1,2,3,",   ",1,2",
      "1,,3",      " 1,2,3",    "1, 2,3",    "1 ,2,3",   "+1,2,3",
      "nan,2,3",   "1,inf,3",   "1,2,-inf",  "1e999,2,3", "1,2,x",
      "1;2;3",     "0x1p3,2,3",
  };
  for (const std::string_view line : bad) {
    CHECK(!ParseGnssCsvLine(line).has_value());
  }
}

// Every double printed with "%.18e", as numpy.savetxt does, reads back as the
// same double.
void TestPythonPrecisionRoundTrips() {
  const std::array<double, 9> values = {
      0.1,
      -4.582129672942816256,
      46.171861679,
      1.0 / 3.0,
      std::nextafter(1.0, 2.0),
      -1e-300,
      std::numeric_limits<double>::denorm_min(),
      std::numeric_limits<double>::max(),
      -std::numeric_limits<double>::max(),
  };
  for (const double value : values) {
    std::array<char, 128> text = {};
    std::snprintf(text.data(), text.size(), "%.18e,%.18e,%.18e", value,
                  -value, value);
    const std::optional<GnssFix> fix = ParseGnssCsvLine(text.data());
    CHECK(fix.has_value());
    if (fix.has_value()) {
      CHECK(fix->t == value && fix->east == -value && fix->north == value);
    }
  }
}

void TestParsePythonFile() {
  const GnssCsvResult result = Parse(kPythonCsv);
  CHECK(result.ok());
  CHECK(result.line == 0);
  CHECK(result.fixes.size() == 3);
  if (result.fixes.size() == 3) {
    CHECK(SameFix(result.fixes[0], {.t = 0.5,
                                    .east = 6.060935039844732586,
                                    .north = -4.582129672942816256}));
    CHECK(SameFix(result.fixes[1], {.t = 1.5,
                                    .east = 17.65074825772709488,
                                    .north = -1.576112805849749776}));
    CHECK(SameFix(result.fixes[2], {.t = 2.5,
                                    .east = 19.44205912174712125,
                                    .north = -11.18620235908823268}));
  }
}

// With the header alone the drive has no fixes, which is a valid file.
void TestHeaderOnly() {
  for (const std::string_view text :
       {"t,east,north", "t,east,north\n", "t,east,north\r\n\r\n\n"}) {
    const GnssCsvResult result = Parse(text);
    CHECK(result.ok());
    CHECK(result.fixes.empty());
  }
}

void TestCrlfAndTrailingBlankLines() {
  const GnssCsvResult result = Parse(
      "t,east,north\r\n"
      "0.5,1,2\r\n"
      "1.5,3,4\r\n"
      "\r\n"
      "  \n");
  CHECK(result.ok());
  CHECK(result.fixes.size() == 2);
}

void TestBadHeader() {
  CHECK(FailsWith(Parse(""), GnssCsvError::kBadHeader, 1));
  CHECK(FailsWith(Parse("\n\n"), GnssCsvError::kBadHeader, 1));
  CHECK(FailsWith(Parse("0.5,1,2\n"), GnssCsvError::kBadHeader, 1));
  CHECK(FailsWith(Parse("t,north,east\n0.5,1,2\n"), GnssCsvError::kBadHeader,
                  1));
  CHECK(FailsWith(Parse("t,east\n"), GnssCsvError::kBadHeader, 1));
  CHECK(FailsWith(Parse(" t,east,north\n"), GnssCsvError::kBadHeader, 1));
  CHECK(FailsWith(Parse("# t,east,north\n"), GnssCsvError::kBadHeader, 1));
  // A filter estimate file is not a GNSS file.
  CHECK(FailsWith(Parse("t,east,north,yaw,v,std_east,std_north,updated\n"),
                  GnssCsvError::kBadHeader, 1));
}

void TestBadLine() {
  CHECK(FailsWith(Parse("t,east,north\n0.5,1,2\n1.5,3\n"),
                  GnssCsvError::kBadLine, 3));
  CHECK(FailsWith(Parse("t,east,north\nnan,1,2\n"), GnssCsvError::kBadLine,
                  2));
  // A blank line counts only at the end of the file.
  CHECK(FailsWith(Parse("t,east,north\n0.5,1,2\n\n1.5,3,4\n"),
                  GnssCsvError::kBadLine, 3));
  // A second header.
  CHECK(FailsWith(Parse("t,east,north\n0.5,1,2\nt,east,north\n"),
                  GnssCsvError::kBadLine, 3));
}

void TestTimeNotIncreasing() {
  CHECK(FailsWith(Parse("t,east,north\n0.5,1,2\n0.5,3,4\n"),
                  GnssCsvError::kTimeNotIncreasing, 3));
  CHECK(FailsWith(Parse("t,east,north\n0.5,1,2\n1.5,3,4\n1,5,6\n"),
                  GnssCsvError::kTimeNotIncreasing, 4));
  // Negative times are allowed while they increase.
  const GnssCsvResult negative = Parse("t,east,north\n-1,0,0\n-0.5,0,0\n");
  CHECK(negative.ok() && negative.fixes.size() == 2);
}

void TestReadFile() {
  const TempFile file(kPythonCsv);
  CHECK(!file.path().empty());
  if (!file.path().empty()) {
    const GnssCsvResult result = ReadGnssCsv(file.path());
    CHECK(result.ok());
    CHECK(result.fixes.size() == 3);
  }

  const TempFile header_only(kGnssCsvHeader);
  if (!header_only.path().empty()) {
    const GnssCsvResult result = ReadGnssCsv(header_only.path());
    CHECK(result.ok() && result.fixes.empty());
  }

  CHECK(FailsWith(ReadGnssCsv(""), GnssCsvError::kNoFile, 0));
  if (!file.path().empty()) {
    CHECK(FailsWith(ReadGnssCsv(file.path() + "_missing"),
                    GnssCsvError::kNoFile, 0));
  }
  // A directory is not a GNSS file.
  CHECK(!ReadGnssCsv("/").ok());

  // A stream that fails while it is read is reported as kNoFile, not as an
  // empty file with a bad header.
  std::istringstream broken{std::string(kPythonCsv)};
  broken.setstate(std::ios::badbit);
  CHECK(FailsWith(ParseGnssCsv(broken), GnssCsvError::kNoFile, 0));
}

void TestErrorNames() {
  const std::array<GnssCsvError, 5> errors = {
      GnssCsvError::kNone, GnssCsvError::kNoFile, GnssCsvError::kBadHeader,
      GnssCsvError::kBadLine, GnssCsvError::kTimeNotIncreasing};
  for (std::size_t i = 0; i < errors.size(); ++i) {
    CHECK(ToString(errors[i]) != "unknown error");
    for (std::size_t j = 0; j < i; ++j) {
      CHECK(ToString(errors[i]) != ToString(errors[j]));
    }
  }
}

}  // namespace

int main() {
  TestParseLine();
  TestParseLineRejects();
  TestPythonPrecisionRoundTrips();
  TestParsePythonFile();
  TestHeaderOnly();
  TestCrlfAndTrailingBlankLines();
  TestBadHeader();
  TestBadLine();
  TestTimeNotIncreasing();
  TestReadFile();
  TestErrorNames();

  if (failure_count != 0) {
    std::cerr << failure_count << " checks failed\n";
    return 1;
  }

  std::cout << "All tests passed\n";
  return 0;
}
