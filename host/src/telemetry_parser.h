#ifndef MOTOR_MONITOR_TELEMETRY_PARSER_H_
#define MOTOR_MONITOR_TELEMETRY_PARSER_H_

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

struct MotorFrame {
  std::uint32_t board_ms = 0;
  int motor_id = 0;
  bool online = false;
  double target_deg = 0;
  double angle_deg = 0;
  int rpm = 0;
  int command_current = 0;
  int feedback_current = 0;
  int temperature = 0;
  int encoder = 0;
};

std::optional<MotorFrame> ParseMotorLine(std::string_view line);

// A readyRead callback may contain a partial line or several complete lines.
class TelemetryParser {
 public:
  std::vector<MotorFrame> Feed(std::string_view bytes);
  void Reset();
  std::uint64_t invalid_lines() const { return invalid_lines_; }

 private:
  std::string pending_;
  bool discarding_ = false;
  std::uint64_t invalid_lines_ = 0;
};

#endif  // MOTOR_MONITOR_TELEMETRY_PARSER_H_
