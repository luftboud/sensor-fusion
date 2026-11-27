#ifndef SENSOR_FUSION_IO_TEXT_H_
#define SENSOR_FUSION_IO_TEXT_H_

#include <charconv>
#include <cmath>
#include <cstddef>
#include <optional>
#include <string_view>
#include <system_error>

// Small text helpers shared by the readers in sensor_fusion/io/.

namespace sensor_fusion::internal {

// Returns `s` without trailing spaces, tabs, CR and LF.
inline std::string_view StripTrailingWhitespace(std::string_view s) {
  const std::size_t last = s.find_last_not_of(" \t\r\n");
  if (last == std::string_view::npos) {
    return {};
  }
  return s.substr(0, last + 1);
}

// Parses the whole field as a finite double. Rejects an empty field, leading
// or trailing characters, a leading '+', "inf", "nan", and values out of the
// range of double.
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

}  // namespace sensor_fusion::internal

#endif  // SENSOR_FUSION_IO_TEXT_H_
