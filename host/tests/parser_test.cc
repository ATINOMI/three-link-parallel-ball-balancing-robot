#include "telemetry_parser.h"
#include <cstdlib>
#include <iostream>

void Check(bool condition) {
  if (!condition) { std::cerr << "FAILED\n"; std::exit(1); }
}
int main() {
  const std::string line = "t=1200 M1 online=1 target=10.00 angle=-0.25 rpm=-12 cmd=-35 fb=28 temp=32 ecd=4321\r\n";
  auto frame = ParseMotorLine(line);
  Check(frame && frame->motor_id == 1 && frame->angle_deg == -0.25 && frame->rpm == -12);
  TelemetryParser parser;
  Check(parser.Feed(line.substr(0, 20)).empty());
  auto frames = parser.Feed(line.substr(20) + line + line);
  Check(frames.size() == 3);
  Check(!ParseMotorLine("t=1 M4 online=1 target=0 angle=0 rpm=0 cmd=0 fb=0 temp=0 ecd=0"));
  Check(!ParseMotorLine("t=1 M2 online=1 target=nan angle=0 rpm=0 cmd=0 fb=0 temp=0 ecd=0"));
  Check(!ParseMotorLine("t=1 M2 online=1 target=0 angle=0 rpm=0 cmd=0 fb=0 temp=0 ecd=8192"));
  Check(!ParseMotorLine("t=1 M2 online=1 target=0 angle=0 rpm=0 cmd=0 fb=0 temp=0"));
  Check(!ParseMotorLine("t=1 M2 online=1 target=0 angle=0 rpm=0 cmd=0 fb=0 temp=0 ecd=1 t=2"));
  frames = parser.Feed(std::string(5000, 'x') + "\n" + line);
  Check(frames.size() == 1 && parser.invalid_lines() == 1);
  parser.Reset();
  Check(parser.invalid_lines() == 0);
  auto offline = ParseMotorLine("t=4294967295 M3 online=0 target=360 angle=359.9 rpm=0 cmd=0 fb=-1 temp=35 ecd=8191");
  Check(offline && !offline->online && offline->board_ms == 4294967295U);
  std::cout << "PASS: framing, multiple lines, signed data, validation, overflow resynchronization, offline data.\n";
}
