#ifndef MOTOR_MONITOR_MOTOR_INPUT_H_
#define MOTOR_MONITOR_MOTOR_INPUT_H_

struct ControlButtons {
  bool up = false;
  bool down = false;
  bool l1 = false;
  bool r1 = false;
};

struct MotorAction {
  int selection_delta = 0;
  int step_degrees = 0;
};

// One action per rising edge. Seed with the current snapshot when connecting.
class MotorInput {
 public:
  void SetBaseline(ControlButtons buttons) { previous_ = buttons; }
  MotorAction Update(ControlButtons buttons) {
    MotorAction action;
    if (buttons.l1 != buttons.r1) {
      if (buttons.l1 && !previous_.l1) action.selection_delta = -1;
      if (buttons.r1 && !previous_.r1) action.selection_delta = 1;
    }
    if (!buttons.l1 && !buttons.r1 && buttons.up != buttons.down) {
      if (buttons.up && !previous_.up) action.step_degrees = 10;
      if (buttons.down && !previous_.down) action.step_degrees = -10;
    }
    previous_ = buttons;
    return action;
  }

 private:
  ControlButtons previous_;
};

inline int NextMotor(int current, int direction) {
  return (current - 1 + direction + 3) % 3 + 1;
}

inline int PlayerLightMask(int motor_id) {
  constexpr int kMasks[] = {0x04, 0x0a, 0x15};
  return motor_id >= 1 && motor_id <= 3 ? kMasks[motor_id - 1] : 0;
}

#endif  // MOTOR_MONITOR_MOTOR_INPUT_H_
