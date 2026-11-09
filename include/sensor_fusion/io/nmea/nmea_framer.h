#ifndef SENSOR_FUSION_IO_NMEA_NMEA_FRAMER_H_
#define SENSOR_FUSION_IO_NMEA_NMEA_FRAMER_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace sensor_fusion {

// Byte-by-byte NMEA 0183 framer. Finds sentences in a byte stream from a log
// file or a UART and checks their checksums. It allocates no memory and
// throws no exceptions.
//
//   NmeaParser parser;
//   for (const char c : data) {
//     if (parser.ReceiveByte(static_cast<unsigned char>(c))) {
//       Use(parser.sentence());
//     }
//   }
class NmeaParser {
 public:
  // Most bytes between '$' and '*'. NMEA 0183 limits a whole sentence to 82
  // characters, counting '$', "*hh" and CR LF, which leaves 82 - 6 = 76.
  static constexpr std::size_t kMaxBodySize = 76;

  // Returns the last framed sentence without '$', '*', the checksum and the
  // line end. The view points into the parser and stays valid until the next
  // '$', so parse or copy it right after ReceiveByte() returns true.
  [[nodiscard]] std::string_view sentence() const {
    return {buffer_.data(), size_};
  }

  // Feeds one byte. Returns true if this byte completed a sentence whose
  // checksum matches.
  bool ReceiveByte(unsigned char byte) {
    if (byte == '$') {
      size_ = 0;
      checksum_ = 0;
      state_ = State::kInnerChar;
      return false;
    }

    switch (state_) {
      case State::kWaitingChar:
        return false;

      case State::kInnerChar:
        if (byte == '*') {
          expected_checksum_digits_ = 0;
          expected_checksum_ = 0;
          state_ = State::kCheckStarChar;
        } else if (size_ < buffer_.size() && byte >= 0x20 && byte <= 0x7E) {
          buffer_[size_] = static_cast<char>(byte);
          ++size_;
          checksum_ ^= byte;
        } else {
          state_ = State::kWaitingChar;
        }
        return false;

      case State::kCheckStarChar: {
        const int value = HexValue(static_cast<char>(byte));
        if (value < 0) {
          state_ = State::kWaitingChar;
          return false;
        }
        if (expected_checksum_digits_ == 0) {
          expected_checksum_ = static_cast<std::uint8_t>(value * 16);
        } else {
          expected_checksum_ =
              static_cast<std::uint8_t>(expected_checksum_ + value);
          state_ = State::kNewRowChar;
        }
        ++expected_checksum_digits_;
        return false;
      }

      case State::kNewRowChar:
        if (byte == '\r') {
          return false;
        }
        state_ = State::kWaitingChar;
        if (byte == '\n') {
          return checksum_ == expected_checksum_;
        }
        return false;
    }
    return false;
  }

 private:
  enum class State : std::uint8_t {
    kWaitingChar,    // Waits for '$' and ignores every other byte.
    kInnerChar,      // Stores body bytes and updates the checksum.
    kCheckStarChar,  // Reads the two hex digits after '*'.
    kNewRowChar,     // Skips '\r' and compares the checksums on '\n'.
  };

  // Returns the value of a hex digit in either case, or -1.
  static int HexValue(char c) {
    if (c >= '0' && c <= '9') {
      return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
      return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
      return c - 'a' + 10;
    }
    return -1;
  }

  std::array<char, kMaxBodySize> buffer_ = {};
  State state_ = State::kWaitingChar;
  std::uint8_t checksum_ = 0;
  std::uint8_t expected_checksum_ = 0;
  std::uint8_t expected_checksum_digits_ = 0;
  std::size_t size_ = 0;
};

}  // namespace sensor_fusion

#endif  // SENSOR_FUSION_IO_NMEA_NMEA_FRAMER_H_
