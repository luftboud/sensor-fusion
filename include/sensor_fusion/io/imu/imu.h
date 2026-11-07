#ifndef SENSOR_FUSION_IO_IMU_IMU_H_
#define SENSOR_FUSION_IO_IMU_IMU_H_

#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "sensor_fusion/io/nmea/nmea_tokeniser.h"


namespace sensor_fusion {

// One IMU measurement in the vehicle frame.
struct ImuSample {
  double t = 0.0;                    // Seconds since the first sample.
  std::array<double, 3> accel = {};  // m/s^2, specific force (with gravity).
  std::array<double, 3> gyro = {};   // rad/s.
};

// One line of a KITTI oxts file: 30 space-separated values in this order.
// Two fixes to dataformat.txt: the third acceleration is az (the file says
// ay), and pos_accuracy is position accuracy in metres (the file says
// velocity accuracy).
struct OxtsRecord {
  static constexpr std::size_t kFieldCount = 30;

  double lat;  // Latitude of the oxts unit (deg).
  double lon;  // Longitude of the oxts unit (deg).
  double alt;  // Altitude of the oxts unit (m).
  // Roll angle (rad): 0 = level, positive = left side up, range -pi..+pi.
  double roll;
  // Pitch angle (rad): 0 = level, positive = front down, range -pi/2..+pi/2.
  double pitch;
  // Heading (rad): 0 = east, positive = counter-clockwise, range -pi..+pi.
  double yaw;
  double vn;  // Velocity towards north (m/s).
  double ve;  // Velocity towards east (m/s).
  double vf;  // Forward velocity, parallel to the earth surface (m/s).
  double vl;  // Leftward velocity, parallel to the earth surface (m/s).
  double vu;  // Upward velocity, perpendicular to the earth surface (m/s).
  double ax;  // Acceleration along x, towards the vehicle front (m/s^2).
  double ay;  // Acceleration along y, towards the vehicle left (m/s^2).
  double az;  // Acceleration along z, towards the vehicle top (m/s^2).
  double af;  // Forward acceleration (m/s^2).
  double al;  // Leftward acceleration (m/s^2).
  double au;  // Upward acceleration (m/s^2).
  double wx;  // Angular rate around x (rad/s).
  double wy;  // Angular rate around y (rad/s).
  double wz;  // Angular rate around z (rad/s).
  double wf;  // Angular rate around the forward axis (rad/s).
  double wl;  // Angular rate around the leftward axis (rad/s).
  double wu;  // Angular rate around the upward axis (rad/s).
  double pos_accuracy;  // Position accuracy, north/east (m).
  double vel_accuracy;  // Velocity accuracy, north/east (m/s).
  int navstat;          // Navigation status.
  int numsats;          // Satellites tracked by the primary GPS receiver.
  int posmode;          // Position mode of the primary GPS receiver.
  int velmode;          // Velocity mode of the primary GPS receiver.
  int orimode;          // Orientation mode of the primary GPS receiver.
};

// Example line:
// 49.009347760599 8.4371344308375 114.45652008057 0.072242 0.001566 -0.2697266732051 -2.9301818443449 10.286884437741 10.695750797331 0.082893715568935 -0.075015404010604 0.0059525410318346 0.845164147132 9.9149729634473 0.041978129261809 0.13652761309179 9.9501668070309 0.016662646247367 -0.036831580656409 0.003311640516573 0.016662387052679 -0.036976147506788 0.00060422104178454 0.42856154750514 0.057982756057297 4 7 4 4 0

// Why ReadKittiOxts() failed.
enum class KittiError : std::uint8_t {
  kNone,
  kNoTimestampsFile,   // oxts/timestamps.txt does not open.
  kNoTimestamps,       // timestamps.txt has no lines.
  kBadTimestamp,       // A timestamps.txt line does not parse.
  kTimeNotIncreasing,  // A timestamp is not greater than the previous one.
  kMissingDataFile,    // data/<i>.txt is missing for the i-th timestamp.
  kBadDataLine,        // A data/<i>.txt line does not parse.
  kExtraDataFile,      // data/ holds a file that has no timestamp.
};

struct KittiReadResult {
  KittiError error = KittiError::kNone;
  std::size_t index = 0;           // Line or file number of the error.
  std::int64_t start_ns = 0;       // Absolute time of the first sample (ns).
  std::vector<ImuSample> samples;  // Empty on error.

  [[nodiscard]] bool ok() const { return error == KittiError::kNone; }
};

namespace internal {

// Returns `s` without trailing spaces, tabs, CR and LF.
inline std::string_view StripTrailingWhitespace(std::string_view s) {
  const std::size_t last = s.find_last_not_of(" \t\r\n");
  if (last == std::string_view::npos) {
    return {};
  }
  return s.substr(0, last + 1);
}

// Parses the whole field as a finite double. Rejects an empty field, trailing
// characters, "inf" and "nan".
inline std::optional<double> ParseDouble(std::string_view field) {
  double value = 0.0;
  const char* const begin = field.data();
  const char* const end = begin + field.size();
  const auto [ptr, ec] =
      std::from_chars(begin, end, value, std::chars_format::general);
  if (ec != std::errc{} || ptr != end || !std::isfinite(value)) {
    return std::nullopt;
  }
  return value;
}

// Parses a whole-valued number that fits in int. KITTI writes these fields
// both as "4" and as "4.00000000000000", so "4.5" is rejected but the second
// form is accepted.
inline std::optional<int> ParseWholeNumber(std::string_view field) {
  const std::optional<double> value = ParseDouble(field);
  if (!value.has_value() || std::trunc(*value) != *value ||
      *value < std::numeric_limits<int>::min() ||
      *value > std::numeric_limits<int>::max()) {
    return std::nullopt;
  }
  return static_cast<int>(*value);
}

// Parses a field made of decimal digits only. A sign, spaces or an empty
// field give std::nullopt.
inline std::optional<std::int64_t> ParseDigits(std::string_view field) {
  if (field.empty()) {
    return std::nullopt;
  }
  for (const char c : field) {
    if (c < '0' || c > '9') {
      return std::nullopt;
    }
  }
  std::int64_t value = 0;
  const char* const begin = field.data();
  const char* const end = begin + field.size();
  const auto [ptr, ec] = std::from_chars(begin, end, value);
  if (ec != std::errc{} || ptr != end) {
    return std::nullopt;
  }
  return value;
}

// Returns "0000000042.txt" for 42.
inline std::string DataFileName(std::size_t index) {
  std::string name = std::to_string(index);
  if (name.size() < 10) {
    name.insert(0, 10 - name.size(), '0');
  }
  return name + ".txt";
}

inline KittiReadResult KittiFailure(KittiError error, std::size_t index) {
  KittiReadResult result;
  result.error = error;
  result.index = index;
  return result;
}

// Returns the first line of the file, or std::nullopt if it does not open.
inline std::optional<std::string> ReadFirstLine(const std::string& path) {
  std::ifstream file(path);
  if (!file) {
    return std::nullopt;
  }
  std::string line;
  std::getline(file, line);
  return line;
}

}  // namespace internal

// Parses one oxts line. Returns std::nullopt unless the line has exactly
// 30 finite numbers and the last five are whole numbers.
inline std::optional<OxtsRecord> ParseOxtsLine(std::string_view line) {
  const std::optional<NmeaFields> fields =
      SplitFields(internal::StripTrailingWhitespace(line), ' ');
  if (!fields.has_value() || fields->size() != OxtsRecord::kFieldCount) {
    return std::nullopt;
  }

  constexpr std::size_t kRealCount = 25;
  std::array<double, kRealCount> reals = {};
  for (std::size_t i = 0; i < kRealCount; ++i) {
    const std::optional<double> value =
        internal::ParseDouble(fields->field(i));
    if (!value.has_value()) {
      return std::nullopt;
    }
    reals[i] = *value;
  }

  std::array<int, OxtsRecord::kFieldCount - kRealCount> codes = {};
  for (std::size_t i = 0; i < codes.size(); ++i) {
    const std::optional<int> value =
        internal::ParseWholeNumber(fields->field(kRealCount + i));
    if (!value.has_value()) {
      return std::nullopt;
    }
    codes[i] = *value;
  }

  // C++20 designated initializers: the compiler checks that the fields are
  // listed in the same order as in the struct.
  return OxtsRecord{
      .lat = reals[0],
      .lon = reals[1],
      .alt = reals[2],
      .roll = reals[3],
      .pitch = reals[4],
      .yaw = reals[5],
      .vn = reals[6],
      .ve = reals[7],
      .vf = reals[8],
      .vl = reals[9],
      .vu = reals[10],
      .ax = reals[11],
      .ay = reals[12],
      .az = reals[13],
      .af = reals[14],
      .al = reals[15],
      .au = reals[16],
      .wx = reals[17],
      .wy = reals[18],
      .wz = reals[19],
      .wf = reals[20],
      .wl = reals[21],
      .wu = reals[22],
      .pos_accuracy = reals[23],
      .vel_accuracy = reals[24],
      .navstat = codes[0],
      .numsats = codes[1],
      .posmode = codes[2],
      .velmode = codes[3],
      .orimode = codes[4],
  };
}

inline ImuSample ToImuSample(double t, const OxtsRecord& record) {
  return ImuSample{
      .t = t,
      .accel = {record.ax, record.ay, record.az},
      .gyro = {record.wx, record.wy, record.wz},
  };
}

// Converts "2011-09-26 13:08:25.077436928" to nanoseconds since
// 1970-01-01 00:00:00. The fraction may have 1 to 9 digits.
inline std::optional<std::int64_t> ParseKittiTimestamp(std::string_view line) {
  const std::string_view s = internal::StripTrailingWhitespace(line);

  // YYYY-MM-DD HH:MM:SS.f
  if (s.size() < 21 || s.size() > 29) {
    return std::nullopt;
  }
  if (s[4] != '-' || s[7] != '-' || s[10] != ' ' || s[13] != ':' ||
      s[16] != ':' || s[19] != '.') {
    return std::nullopt;
  }

  const auto year = internal::ParseDigits(s.substr(0, 4));
  const auto month = internal::ParseDigits(s.substr(5, 2));
  const auto day = internal::ParseDigits(s.substr(8, 2));
  const auto hour = internal::ParseDigits(s.substr(11, 2));
  const auto minute = internal::ParseDigits(s.substr(14, 2));
  const auto second = internal::ParseDigits(s.substr(17, 2));
  const std::string_view fraction = s.substr(20);
  const auto fraction_value = internal::ParseDigits(fraction);
  if (!year || !month || !day || !hour || !minute || !second ||
      !fraction_value) {
    return std::nullopt;
  }

  // year_month_day::ok() checks the month, and the day against leap years.
  const std::chrono::year_month_day date{
      std::chrono::year{static_cast<int>(*year)},
      std::chrono::month{static_cast<unsigned>(*month)},
      std::chrono::day{static_cast<unsigned>(*day)},
  };
  if (!date.ok()) {
    return std::nullopt;
  }
  if (*hour > 23 || *minute > 59 || *second > 59) {
    return std::nullopt;
  }

  // ".5" is 500000000 ns.
  std::int64_t fraction_ns = *fraction_value;
  for (std::size_t i = fraction.size(); i < 9; ++i) {
    fraction_ns *= 10;
  }

  const std::int64_t seconds_of_day = (*hour * 3600) + (*minute * 60) + *second;
  const std::chrono::nanoseconds time_of_day{
      (seconds_of_day * 1'000'000'000) + fraction_ns};

  // sys_days counts days since 1970-01-01.
  const auto moment = std::chrono::sys_days{date} + time_of_day;
  return static_cast<std::int64_t>(moment.time_since_epoch().count());
}

inline std::string_view ToString(KittiError error) {
  switch (error) {
    case KittiError::kNone:
      return "no error";
    case KittiError::kNoTimestampsFile:
      return "timestamps.txt not found";
    case KittiError::kNoTimestamps:
      return "timestamps.txt is empty";
    case KittiError::kBadTimestamp:
      return "malformed timestamp";
    case KittiError::kTimeNotIncreasing:
      return "timestamp is not greater than the previous one";
    case KittiError::kMissingDataFile:
      return "data file not found";
    case KittiError::kBadDataLine:
      return "malformed data line";
    case KittiError::kExtraDataFile:
      return "data file without a timestamp";
  }
  return "unknown error";
}

// Reads a KITTI oxts directory: timestamps.txt plus data/0000000000.txt and
// onwards, one file per timestamp. Sample times are relative to the first
// timestamp.
inline KittiReadResult ReadKittiOxts(const std::string& oxts_dir) {
  std::ifstream timestamps_file(oxts_dir + "/timestamps.txt");
  if (!timestamps_file) {
    return internal::KittiFailure(KittiError::kNoTimestampsFile, 0);
  }

  std::vector<std::string> timestamp_lines;
  for (std::string line; std::getline(timestamps_file, line);) {
    timestamp_lines.push_back(std::move(line));
  }
  // Drop blank lines at the end of the file.
  while (!timestamp_lines.empty() &&
         internal::StripTrailingWhitespace(timestamp_lines.back()).empty()) {
    timestamp_lines.pop_back();
  }
  if (timestamp_lines.empty()) {
    return internal::KittiFailure(KittiError::kNoTimestamps, 0);
  }

  const std::string data_dir = oxts_dir + "/data/";
  KittiReadResult result;
  result.samples.reserve(timestamp_lines.size());

  std::int64_t previous_ns = 0;
  for (std::size_t i = 0; i < timestamp_lines.size(); ++i) {
    const std::optional<std::int64_t> ns =
        ParseKittiTimestamp(timestamp_lines[i]);
    if (!ns.has_value()) {
      return internal::KittiFailure(KittiError::kBadTimestamp, i);
    }
    if (i == 0) {
      result.start_ns = *ns;
    } else if (*ns <= previous_ns) {
      return internal::KittiFailure(KittiError::kTimeNotIncreasing, i);
    }
    previous_ns = *ns;

    const std::optional<std::string> data_line =
        internal::ReadFirstLine(data_dir + internal::DataFileName(i));
    if (!data_line.has_value()) {
      return internal::KittiFailure(KittiError::kMissingDataFile, i);
    }
    const std::optional<OxtsRecord> record = ParseOxtsLine(*data_line);
    if (!record.has_value()) {
      return internal::KittiFailure(KittiError::kBadDataLine, i);
    }

    const double t = static_cast<double>(*ns - result.start_ns) * 1e-9;
    result.samples.push_back(ToImuSample(t, *record));
  }

  // There must be no more data files than timestamps.
  const std::size_t next = timestamp_lines.size();
  if (std::ifstream(data_dir + internal::DataFileName(next)).is_open()) {
    return internal::KittiFailure(KittiError::kExtraDataFile, next);
  }

  return result;
}

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_IO_IMU_IMU_H_
