#include "main_window.h"

#include <SDL3/SDL.h>
#include <QApplication>
#include <QComboBox>
#include <QPushButton>
#include <algorithm>
#include <cstdlib>
#include <iostream>

namespace {
void Check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}
}  // namespace

class MainWindowTestPeer {
 public:
  // Simulate a live transport without opening a serial port or driving motors.
  static void UseSimulatedTransport(MainWindow& window) { window.demo_ = true; }
  static void Feedback(MainWindow& window, int id, bool online) {
    MotorFrame frame;
    frame.motor_id = id;
    frame.online = online;
    window.UpdateFrame(frame);
  }
  static void Poll(MainWindow& window) { window.PollGamepad(); }
  static double Target(const MainWindow& window, int id) {
    return window.demo_targets_[id - 1];
  }
  static int Selected(const MainWindow& window) { return window.selected_motor_; }
};

int main(int argc, char* argv[]) {
  QApplication application(argc, argv);
  dualforge::Context context;
  Check(static_cast<bool>(context.status()), "driver initialization");
  SDL_VirtualJoystickDesc descriptor{};
  SDL_INIT_INTERFACE(&descriptor);
  descriptor.type = SDL_JOYSTICK_TYPE_GAMEPAD;
  descriptor.vendor_id = 0x054c;
  descriptor.product_id = 0x0ce6;
  descriptor.name = "Discovery regression DualSense";
  descriptor.naxes = SDL_GAMEPAD_AXIS_COUNT;
  descriptor.nbuttons = SDL_GAMEPAD_BUTTON_COUNT;
  descriptor.axis_mask = (1u << SDL_GAMEPAD_AXIS_COUNT) - 1;
  descriptor.button_mask = (1u << SDL_GAMEPAD_BUTTON_COUNT) - 1;
  const auto id = SDL_AttachVirtualJoystick(&descriptor);
  Check(id != 0, "virtual controller attachment");
  Check(static_cast<bool>(context.poll()), "controller enumeration refresh");
  const auto devices = context.enumerate();
  const auto device = std::find_if(devices.begin(), devices.end(),
      [id](const auto& info) { return info.id == id; });
  Check(device != devices.end() && !device->usb,
        "driver exposes connection type as unknown before open");
  {
    MainWindow window;
    auto* combo = window.findChild<QComboBox*>("gamepadDevices");
    auto* button = window.findChild<QPushButton*>("gamepadConnectButton");
    Check(combo && button, "gamepad controls exist in Qt window");
    const int index = combo->findData(static_cast<quint32>(id));
    Check(index >= 0, "unknown connection type must remain discoverable");
    combo->setCurrentIndex(index);
    Check(button->isEnabled(), "connection button enabled for discovered device");
    button->click();
    Check(button->text() == "断开 PS5", "Qt can open discovered controller");
    MainWindowTestPeer::UseSimulatedTransport(window);
    for (int motor = 1; motor <= 3; ++motor) {
      MainWindowTestPeer::Feedback(window, motor, true);
    }
    auto* joystick = SDL_GetJoystickFromID(id);
    Check(joystick != nullptr, "virtual joystick is open");
    auto press = [&](SDL_GamepadButton key, bool down) {
      Check(SDL_SetJoystickVirtualButton(joystick, key, down), "virtual button update");
    };
    auto poll = [&] { MainWindowTestPeer::Poll(window); };
    press(SDL_GAMEPAD_BUTTON_DPAD_UP, true);
    poll();
    Check(MainWindowTestPeer::Target(window, 1) == 10,
          "first press immediately after connect/readiness must produce +10");
    poll();
    Check(MainWindowTestPeer::Target(window, 1) == 10, "held key must not repeat");
    press(SDL_GAMEPAD_BUTTON_DPAD_UP, false);
    poll();
    press(SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER, true);
    poll();
    Check(MainWindowTestPeer::Selected(window) == 2, "R1 selects motor 2");
    // Release R1 and press UP before another neutral poll can occur.
    press(SDL_GAMEPAD_BUTTON_RIGHT_SHOULDER, false);
    press(SDL_GAMEPAD_BUTTON_DPAD_UP, true);
    poll();
    Check(MainWindowTestPeer::Target(window, 2) == 10,
          "first press immediately after switching must not be swallowed");
    press(SDL_GAMEPAD_BUTTON_DPAD_UP, false);
    poll();
    MainWindowTestPeer::Feedback(window, 2, false);
    MainWindowTestPeer::Feedback(window, 2, true);
    press(SDL_GAMEPAD_BUTTON_DPAD_UP, true);
    poll();
    Check(MainWindowTestPeer::Target(window, 2) == 20,
          "first press immediately after feedback restoration must produce +10");
    // A press observed while offline is ignored and never replayed on recovery.
    press(SDL_GAMEPAD_BUTTON_DPAD_UP, false);
    poll();
    MainWindowTestPeer::Feedback(window, 2, false);
    press(SDL_GAMEPAD_BUTTON_DPAD_UP, true);
    poll();
    MainWindowTestPeer::Feedback(window, 2, true);
    poll();
    Check(MainWindowTestPeer::Target(window, 2) == 20,
          "key held through feedback recovery must not move the motor");
    auto* step = window.findChild<QPushButton*>("stepUpButton");
    Check(step && step->isEnabled(), "fresh feedback enables Qt button immediately");
    step->click();
    Check(MainWindowTestPeer::Target(window, 2) == 30, "one Qt click produces +10");
    MainWindowTestPeer::Feedback(window, 2, false);
    Check(!step->isEnabled(), "offline feedback disables Qt button immediately");
    MainWindowTestPeer::Feedback(window, 2, true);
    Check(step->isEnabled(), "recovered feedback enables first Qt click immediately");
    step->click();
    Check(MainWindowTestPeer::Target(window, 2) == 40, "first recovered Qt click produces +10");
    button->click();
    Check(button->text() == "连接 PS5", "Qt can disconnect controller");
  }
  Check(SDL_DetachVirtualJoystick(id), "virtual controller cleanup");
  std::cout << "PASS: driver/Qt discovery, connect/disconnect, immediate first presses, "
               "switch/recovery, hold suppression and immediate Qt buttons\n";
}
