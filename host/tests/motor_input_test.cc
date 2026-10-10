#include "motor_input.h"

#include <cstdlib>
#include <iostream>

void Check(bool condition) {
  if (!condition) std::exit(1);
}

int main() {
  MotorInput input;
  input.SetBaseline({true, false, false, false});
  Check(input.Update({true, false, false, false}).step_degrees == 0);
  Check(input.Update({}).step_degrees == 0);
  Check(input.Update({true, false, false, false}).step_degrees == 10);
  for (int i = 0; i < 500; ++i) {
    Check(input.Update({true, false, false, false}).step_degrees == 0);
  }
  input.Update({});
  Check(input.Update({false, true, false, false}).step_degrees == -10);
  input.Update({});
  Check(input.Update({true, true, false, false}).step_degrees == 0);
  input.Update({});
  auto action = input.Update({true, false, false, true});
  Check(action.selection_delta == 1 && action.step_degrees == 0);
  Check(input.Update({true, false, false, false}).step_degrees == 0);
  input.Update({});
  Check(input.Update({false, false, true, false}).selection_delta == -1);
  input.Update({});
  Check(input.Update({false, false, true, true}).selection_delta == 0);
  input.SetBaseline({false, true, false, true});
  Check(input.Update({false, true, false, true}).step_degrees == 0);
  input.SetBaseline({});
  Check(input.Update({false, true, false, false}).step_degrees == -10);
  Check(NextMotor(1, -1) == 3 && NextMotor(3, 1) == 1);
  Check(PlayerLightMask(1) == 4 && PlayerLightMask(2) == 10 &&
        PlayerLightMask(3) == 21 && PlayerLightMask(0) == 0);
  std::cout << "PASS: button edges, reconnect latching, opposite keys, "
               "switch precedence, cyclic selection and player LEDs\n";
}
