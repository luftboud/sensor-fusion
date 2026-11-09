#ifndef SENSOR_FUSION_IO_NMEA_NMEA_TOKENISER_H_
#define SENSOR_FUSION_IO_NMEA_NMEA_TOKENISER_H_

#include <array>
#include <cstddef>
#include <optional>
#include <string_view>

namespace sensor_fusion {

// Most fields SplitFields() keeps. GSV in NMEA 4.10 has 21 fields and a KITTI
// oxts line has 30.
inline constexpr std::size_t kMaxFields = 32;

// Fields of one sentence. Each field is a view into the string that was split.
class NmeaFields {
 public:
  // Appends a field. Returns false, and keeps the fields unchanged, when
  // kMaxFields fields are already stored.
  bool Push(std::string_view field) {
    if (size_ >= fields_.size()) {
      return false;
    }
    fields_[size_] = field;
    ++size_;
    return true;
  }

  // Returns the field at `index`, or an empty view past the last field.
  [[nodiscard]] std::string_view field(std::size_t index) const {
    if (index >= size_) {
      return {};
    }
    return fields_[index];
  }

  [[nodiscard]] std::size_t size() const { return size_; }

 private:
  std::array<std::string_view, kMaxFields> fields_ = {};
  std::size_t size_ = 0;
};

// Splits `sentence` at every `delimiter`: n delimiters give exactly n + 1
// fields. An empty field is a zero-length view that keeps its position.
// Nothing is trimmed or validated. Use ',' for NMEA and ' ' for KITTI.
//
// The fields point into `sentence` without copies. For a sentence taken from
// NmeaParser::sentence() they stay valid only until the parser sees the next
// '$'.
//
// Returns std::nullopt when there are more than kMaxFields fields.
inline std::optional<NmeaFields> SplitFields(std::string_view sentence,
                                             char delimiter = ',') {
  NmeaFields result;

  std::size_t start = 0;
  std::size_t end = sentence.find(delimiter);

  while (end != std::string_view::npos) {
    if (!result.Push(sentence.substr(start, end - start))) {
      return std::nullopt;
    }
    start = end + 1;
    end = sentence.find(delimiter, start);
  }

  if (!result.Push(sentence.substr(start))) {
    return std::nullopt;
  }
  return result;
}

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_IO_NMEA_NMEA_TOKENISER_H_
