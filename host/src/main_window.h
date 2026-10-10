#ifndef MOTOR_MONITOR_MAIN_WINDOW_H_
#define MOTOR_MONITOR_MAIN_WINDOW_H_

#include <array>
#include <memory>
#include <dualforge/dualforge.hpp>
#include <QElapsedTimer>
#include <QMainWindow>
#include <QSerialPort>
#include <QTimer>
#include "telemetry_parser.h"
#include "motor_input.h"

class QComboBox;
class QLabel;
class QPushButton;
class QPlainTextEdit;
class MotorCard;

class MainWindow : public QMainWindow {
 public:
  explicit MainWindow(bool start_demo = false);
  ~MainWindow() override;

 private:
  friend class MainWindowTestPeer;
  void RefreshPorts();
  void ToggleConnection();
  void Disconnect(const QString& reason);
  void Receive();
  void UpdateFrame(const MotorFrame& frame);
  void UpdateStatus();
  void ToggleDemo();
  void AppendLog(const QString& text);
  void RefreshGamepads();
  void ToggleGamepad();
  void CloseGamepad(const QString& reason);
  void PollGamepad();
  void SelectMotor(int direction);
  void UpdateSelection();
  void SetPlayerLights();
  void StepMotor(int degrees);
  bool CanStep() const;

  QSerialPort serial_;
  QTimer status_timer_;
  QTimer demo_timer_;
  QTimer gamepad_timer_;
  QElapsedTimer clock_;
  TelemetryParser parser_;
  std::array<MotorCard*, 3> cards_{};
  QComboBox* ports_ = nullptr;
  QPushButton* connect_button_ = nullptr;
  QPushButton* refresh_button_ = nullptr;
  QPushButton* demo_button_ = nullptr;
  QLabel* connection_status_ = nullptr;
  QLabel* counters_ = nullptr;
  QPlainTextEdit* log_ = nullptr;
  QComboBox* gamepads_ = nullptr;
  QPushButton* gamepad_connect_ = nullptr;
  QPushButton* gamepad_refresh_ = nullptr;
  QLabel* gamepad_status_ = nullptr;
  QLabel* battery_ = nullptr;
  QLabel* selected_label_ = nullptr;
  QLabel* lights_label_ = nullptr;
  QLabel* control_status_ = nullptr;
  std::array<QLabel*, 4> key_labels_{};
  QPushButton* step_up_ = nullptr;
  QPushButton* step_down_ = nullptr;
  std::unique_ptr<dualforge::Context> gamepad_context_;
  std::shared_ptr<dualforge::Controller> gamepad_;
  MotorInput motor_input_;
  int selected_motor_ = 1;
  bool lights_ok_ = false;
  std::array<qint64, 3> arrivals_{{-1, -1, -1}};
  std::array<bool, 3> online_{};
  std::array<double, 3> demo_targets_{};
  bool demo_ = false;
  bool closing_ = false;
  std::uint64_t received_ = 0;
  int demo_tick_ = 0;
};

#endif  // MOTOR_MONITOR_MAIN_WINDOW_H_
