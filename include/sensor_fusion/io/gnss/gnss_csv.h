#ifndef SENSOR_FUSION_IO_GNSS_GNSS_CSV_H_
#define SENSOR_FUSION_IO_GNSS_GNSS_CSV_H_

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <istream>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "sensor_fusion/ekf/scenario.h"
#include "sensor_fusion/io/nmea/nmea_tokeniser.h"
#include "sensor_fusion/io/text.h"

namespace sensor_fusion {

// GNSS fixes in a CSV file, as foreign_impl/ref_ekf.py writes them with
// numpy.savetxt:
//
//   t,east,north
//   5.000000000000000000e-01,6.060935039844732586e+00,-4.582129672942816256e+00
//
// The fields are those of GnssFix: t in seconds on the drive clock, east and
// north in metres from the first reference sample. Python makes the fixes
// and C++ reads them, because the numpy random generator cannot be repeated
// in C++. "%.18e" keeps 19 significant digits, more than the 17 a double
// needs, so every value reads back as the same double.
//
// The first line is the header, exactly "t,east,north". Each other line holds
// three finite numbers separated by commas, without spaces. Times are
// strictly increasing, as RunEkf() requires. A file with the header alone is
// valid and has no fixes, so the whole drive is an outage. CR LF line ends
// and blank lines at the end of the file are accepted.

inline constexpr std::string_view kGnssCsvHeader = "t,east,north";

// Why ReadGnssCsv() or ParseGnssCsv() failed.
enum class GnssCsvError : std::uint8_t {
  kNone,
  kNoFile,             // The file does not open, or reading it fails.
  kBadHeader,          // The first line is missing or not "t,east,north".
  kBadLine,            // A line after the header is not three numbers.
  kTimeNotIncreasing,  // A time is not greater than the time above it.
};

struct GnssCsvResult {
  GnssCsvError error = GnssCsvError::kNone;
  // Line of the error, counted from 1. The header is line 1. 0 for kNoFile.
  std::size_t line = 0;
  std::vector<GnssFix> fixes;  // Empty on error.

  [[nodiscard]] bool ok() const { return error == GnssCsvError::kNone; }
};

namespace internal {

inline GnssCsvResult GnssCsvFailure(GnssCsvError error, std::size_t line) {
  GnssCsvResult result;
  result.error = error;
  result.line = line;
  return result;
}

}  // namespace internal

// Parses one data line such as "0.5,6.06,-4.58". Trailing spaces, tabs and
// CR are ignored. Returns std::nullopt unless the line holds exactly three
// finite numbers.
inline std::optional<GnssFix> ParseGnssCsvLine(std::string_view line) {
  const std::optional<NmeaFields> fields =
      SplitFields(internal::StripTrailingWhitespace(line), ',');
  if (!fields.has_value() || fields->size() != 3) {
    return std::nullopt;
  }
  const std::optional<double> t = internal::ParseDouble(fields->field(0));
  const std::optional<double> east = internal::ParseDouble(fields->field(1));
  const std::optional<double> north = internal::ParseDouble(fields->field(2));
  if (!t.has_value() || !east.has_value() || !north.has_value()) {
    return std::nullopt;
  }
  return GnssFix{.t = *t, .east = *east, .north = *north};
}

// Reads the CSV from an open stream. ReadGnssCsv() opens a file and calls it.
inline GnssCsvResult ParseGnssCsv(std::istream& input) {
  std::vector<std::string> lines;
  for (std::string line; std::getline(input, line);) {
    lines.push_back(std::move(line));
  }
  if (input.bad()) {
    return internal::GnssCsvFailure(GnssCsvError::kNoFile, 0);
  }
  // Drop blank lines at the end of the file.
  while (!lines.empty() &&
         internal::StripTrailingWhitespace(lines.back()).empty()) {
    lines.pop_back();
  }
  if (lines.empty() ||
      internal::StripTrailingWhitespace(lines.front()) != kGnssCsvHeader) {
    return internal::GnssCsvFailure(GnssCsvError::kBadHeader, 1);
  }

  GnssCsvResult result;
  result.fixes.reserve(lines.size() - 1);
  for (std::size_t i = 1; i < lines.size(); ++i) {
    const std::size_t line_number = i + 1;
    const std::optional<GnssFix> fix = ParseGnssCsvLine(lines[i]);
    if (!fix.has_value()) {
      return internal::GnssCsvFailure(GnssCsvError::kBadLine, line_number);
    }
    if (!result.fixes.empty() && fix->t <= result.fixes.back().t) {
      return internal::GnssCsvFailure(GnssCsvError::kTimeNotIncreasing,
                                      line_number);
    }
    result.fixes.push_back(*fix);
  }
  return result;
}

// Reads GNSS fixes from the CSV file at path.
inline GnssCsvResult ReadGnssCsv(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    return internal::GnssCsvFailure(GnssCsvError::kNoFile, 0);
  }
  return ParseGnssCsv(file);
}

inline std::string_view ToString(GnssCsvError error) {
  switch (error) {
    case GnssCsvError::kNone:
      return "no error";
    case GnssCsvError::kNoFile:
      return "GNSS CSV file does not open";
    case GnssCsvError::kBadHeader:
      return "GNSS CSV header is not t,east,north";
    case GnssCsvError::kBadLine:
      return "GNSS CSV line is not three finite numbers";
    case GnssCsvError::kTimeNotIncreasing:
      return "GNSS CSV time is not increasing";
  }
  return "unknown error";
}

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_IO_GNSS_GNSS_CSV_H_
