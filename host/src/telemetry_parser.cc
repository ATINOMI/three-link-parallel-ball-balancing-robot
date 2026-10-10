#include "telemetry_parser.h"

#include <charconv>
#include <cmath>
#include <map>
#include <sstream>

namespace {
template <typename T>
bool Integer(const std::string& text, T* value) {
  auto result = std::from_chars(text.data(), text.data() + text.size(), *value);
  return result.ec == std::errc{} && result.ptr == text.data() + text.size();
}
bool Decimal(const std::string& text, double* value) {
  try {
    std::size_t used = 0;
    *value = std::stod(text, &used);
    return used == text.size() && std::isfinite(*value);
  } catch (...) {
    return false;
  }
}
}  // namespace

std::optional<MotorFrame> ParseMotorLine(std::string_view line) {
  std::istringstream stream{std::string(line)};
  std::map<std::string, std::string> fields;
  std::string token;
  MotorFrame frame;
  bool has_motor = false;
  while (stream >> token) {
    if (token.size() == 2 && token[0] == 'M') {
      if (has_motor || !Integer(token.substr(1), &frame.motor_id)) return {};
      has_motor = true;
    } else {
      auto split = token.find('=');
      if (split == std::string::npos || split == 0 || split + 1 == token.size()) return {};
      if (!fields.emplace(token.substr(0, split), token.substr(split + 1)).second) return {};
    }
  }
  const std::vector<std::string> keys = {
      "t", "online", "target", "angle", "rpm", "cmd", "fb", "temp", "ecd"};
  for (const auto& key : keys) if (fields.count(key) == 0) return {};
  int online = 0;
  if (!has_motor || frame.motor_id < 1 || frame.motor_id > 3 ||
      !Integer(fields["t"], &frame.board_ms) ||
      !Integer(fields["online"], &online) || (online != 0 && online != 1) ||
      !Decimal(fields["target"], &frame.target_deg) ||
      !Decimal(fields["angle"], &frame.angle_deg) ||
      !Integer(fields["rpm"], &frame.rpm) ||
      !Integer(fields["cmd"], &frame.command_current) ||
      !Integer(fields["fb"], &frame.feedback_current) ||
      !Integer(fields["temp"], &frame.temperature) ||
      !Integer(fields["ecd"], &frame.encoder)) return {};
  if (frame.rpm < -32768 || frame.rpm > 32767 ||
      frame.command_current < -32768 || frame.command_current > 32767 ||
      frame.feedback_current < -32768 || frame.feedback_current > 32767 ||
      frame.temperature < 0 || frame.temperature > 255 ||
      frame.encoder < 0 || frame.encoder > 8191) return {};
  frame.online = online == 1;
  return frame;
}

std::vector<MotorFrame> TelemetryParser::Feed(std::string_view bytes) {
  std::vector<MotorFrame> frames;
  for (char ch : bytes) {
    if (ch == '\n') {
      if (!discarding_ && !pending_.empty()) {
        auto frame = ParseMotorLine(pending_);
        if (frame) frames.push_back(*frame);
        else ++invalid_lines_;
      }
      pending_.clear();
      discarding_ = false;
    } else if (!discarding_) {
      if (pending_.size() >= 4096) {
        pending_.clear();
        discarding_ = true;
        ++invalid_lines_;
      } else {
        pending_ += ch;
      }
    }
  }
  return frames;
}

void TelemetryParser::Reset() {
  pending_.clear();
  discarding_ = false;
  invalid_lines_ = 0;
}
