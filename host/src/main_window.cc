#include "main_window.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <QComboBox>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSerialPortInfo>
#include <QScrollArea>
#include <QStyle>
#include <QTextDocument>
#include <QVBoxLayout>

namespace {
QString Number(double value) { return QString::number(value, 'f', 2); }

class AnglePlot : public QWidget {
 public:
  AnglePlot() { setMinimumHeight(110); }
  void Add(double target, double angle) {
    points_.push_back({target, angle});
    if (points_.size() > 120) points_.pop_front();
    update();
  }
  void Clear() { points_.clear(); update(); }

 protected:
  void paintEvent(QPaintEvent*) override {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    QRectF area = rect().adjusted(4, 15, -4, -22);
    p.setPen(QColor("#293449"));
    for (int i = 0; i <= 3; ++i) {
      double y = area.top() + area.height() * i / 3;
      p.drawLine(QPointF(area.left(), y), QPointF(area.right(), y));
    }
    if (points_.empty()) return;
    double low = points_.front().first - 1;
    double high = points_.front().first + 1;
    for (const auto& pair : points_) {
      low = std::min(low, std::min(pair.first, pair.second) - 1);
      high = std::max(high, std::max(pair.first, pair.second) + 1);
    }
    for (int series = 0; series < 2; ++series) {
      QPolygonF poly;
      for (std::size_t i = 0; i < points_.size(); ++i) {
        double value = series == 0 ? points_[i].first : points_[i].second;
        double x = area.left() + area.width() * i / 119;
        double y = area.bottom() - (value - low) * area.height() / (high - low);
        poly << QPointF(x, y);
      }
      p.setPen(QPen(QColor(series == 0 ? "#64748b" : "#4ce0bd"), 2,
                    series == 0 ? Qt::DashLine : Qt::SolidLine));
      if (poly.size() == 1) p.drawEllipse(poly.front(), 2, 2);
      else p.drawPolyline(poly);
    }
  }

 private:
  std::deque<std::pair<double, double>> points_;
};
}  // namespace

class MotorCard : public QFrame {
 public:
  explicit MotorCard(int id) {
    setObjectName("motorCard");
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(22, 22, 22, 22);
    auto* heading = new QHBoxLayout;
    heading->setSpacing(16);
    auto* title = new QLabel(QString("M%1  /  电机 %1").arg(id));
    title->setObjectName("cardTitle");
    status_ = new QLabel("等待数据");
    heading->addWidget(title);
    heading->addStretch();
    heading->addWidget(status_);
    layout->addLayout(heading);
    layout->addSpacing(16);
    auto* caption = new QLabel("输出轴当前角度");
    caption->setObjectName("muted");
    layout->addWidget(caption);
    angle_ = new QLabel("— °");
    angle_->setObjectName("angle");
    layout->addWidget(angle_);
    target_ = new QLabel("目标 — °    误差 — °");
    layout->addWidget(target_);
    auto* plot_caption = new QLabel("角度趋势 · 灰：目标 / 绿：实际");
    plot_caption->setObjectName("muted");
    layout->addWidget(plot_caption);
    plot_ = new AnglePlot;
    layout->addWidget(plot_);
    layout->addSpacing(18);
    auto* grid = new QGridLayout;
    const QStringList names = {"转子转速 / rpm", "指令电流 / raw", "反馈电流 / raw",
                               "电机温度 / °C", "编码器 / count", "板端时间 / ms"};
    for (int i = 0; i < 6; ++i) {
      auto* label = new QLabel(names[i]);
      label->setObjectName("muted");
      label->setMinimumHeight(18);
      values_[i] = new QLabel("—");
      values_[i]->setObjectName("metric");
      values_[i]->setMinimumHeight(30);
      grid->addWidget(label, (i / 2) * 2, i % 2);
      grid->addWidget(values_[i], (i / 2) * 2 + 1, i % 2);
    }
    grid->setVerticalSpacing(8);
    layout->addLayout(grid);
    age_ = new QLabel("尚未收到电机反馈");
    age_->setObjectName("muted");
    layout->addSpacing(10);
    layout->addWidget(age_);
  }

  void Reset() {
    last_arrival_ = -1;
    angle_->setText("— °");
    target_->setText("目标 — °    误差 — °");
    for (auto* value : values_) value->setText("—");
    plot_->Clear();
    SetStatus("等待数据", "#8290a6");
    age_->setText("尚未收到电机反馈");
  }
  void Select(bool selected) {
    setProperty("selected", selected);
    style()->unpolish(this);
    style()->polish(this);
    update();
  }
  void SetFrame(const MotorFrame& frame, qint64 arrival) {
    last_arrival_ = arrival;
    online_ = frame.online;
    angle_->setText(Number(frame.angle_deg) + " °");
    target_->setText("目标 " + Number(frame.target_deg) + " °    误差 " +
                     Number(frame.target_deg - frame.angle_deg) + " °");
    values_[0]->setText(QString::number(frame.rpm));
    values_[1]->setText(QString::number(frame.command_current));
    values_[2]->setText(QString::number(frame.feedback_current));
    values_[3]->setText(QString::number(frame.temperature));
    values_[4]->setText(QString::number(frame.encoder));
    values_[5]->setText(QString::number(frame.board_ms));
    if (frame.online) plot_->Add(frame.target_deg, frame.angle_deg);
  }
  void UpdateAge(qint64 now, bool connected) {
    if (!connected) { SetStatus("未连接", "#8290a6"); return; }
    if (last_arrival_ < 0) { SetStatus("等待数据", "#e5b65a"); return; }
    const auto age = now - last_arrival_;
    age_->setText(QString("距上次串口数据 %1 ms%2").arg(age)
        .arg(online_ ? "" : " · 电机反馈已失效"));
    if (age > 1000) SetStatus("数据超时", "#e5b65a");
    else SetStatus(online_ ? "在线" : "电机离线", online_ ? "#4ce0bd" : "#f07880");
  }

 private:
  void SetStatus(const QString& text, const QString& color) {
    status_->setText("● " + text);
    status_->setStyleSheet("color:" + color + ";font-weight:600;");
  }
  QLabel* status_;
  QLabel* angle_;
  QLabel* target_;
  QLabel* age_;
  std::array<QLabel*, 6> values_{};
  AnglePlot* plot_;
  qint64 last_arrival_ = -1;
  bool online_ = false;
};

MainWindow::MainWindow(bool start_demo) {
  setWindowTitle("PS5 三电机控制台 · USB CDC");
  resize(1280, 900);
  setMinimumSize(1040, 720);
  clock_.start();
  serial_.setReadBufferSize(64 * 1024);
  auto* central = new QWidget;
  auto* scroll = new QScrollArea;
  scroll->setWidgetResizable(true);
  scroll->setFrameShape(QFrame::NoFrame);
  scroll->setWidget(central);
  setCentralWidget(scroll);
  auto* layout = new QVBoxLayout(central);
  layout->setContentsMargins(28, 24, 28, 24);
  layout->setSpacing(12);
  auto* heading = new QLabel("MOTOR / PS5 三电机控制台");
  heading->setObjectName("heading");
  layout->addWidget(heading);
  auto* subtitle = new QLabel("CAN1 电调 1–3  ·  USB CDC  ·  角度单位为减速箱输出轴相对角度");
  subtitle->setObjectName("muted");
  layout->addWidget(subtitle);
  auto* bar = new QHBoxLayout;
  ports_ = new QComboBox;
  ports_->setMinimumWidth(250);
  refresh_button_ = new QPushButton("刷新串口");
  connect_button_ = new QPushButton("连接");
  connect_button_->setObjectName("connectButton");
  demo_button_ = new QPushButton("演示模式");
  connection_status_ = new QLabel("未连接 · 115200 / 8N1");
  bar->addWidget(ports_);
  bar->addWidget(refresh_button_);
  bar->addWidget(connect_button_);
  bar->addWidget(demo_button_);
  bar->addStretch();
  bar->addWidget(connection_status_);
  layout->addLayout(bar);
  auto* panel = new QFrame;
  panel->setObjectName("gamepadPanel");
  auto* pad_layout = new QVBoxLayout(panel);
  pad_layout->setContentsMargins(18, 14, 18, 14);
  auto* pad_bar = new QHBoxLayout;
  gamepads_ = new QComboBox;
  gamepads_->setObjectName("gamepadDevices");
  gamepads_->setMinimumWidth(235);
  gamepad_refresh_ = new QPushButton("刷新手柄");
  gamepad_connect_ = new QPushButton("连接 PS5");
  gamepad_connect_->setObjectName("gamepadConnectButton");
  gamepad_status_ = new QLabel("PS5 未连接");
  battery_ = new QLabel("电量 —");
  pad_bar->addWidget(gamepads_);
  pad_bar->addWidget(gamepad_refresh_);
  pad_bar->addWidget(gamepad_connect_);
  pad_bar->addStretch();
  pad_bar->addWidget(gamepad_status_);
  pad_bar->addWidget(battery_);
  pad_layout->addLayout(pad_bar);
  auto* actions = new QHBoxLayout;
  selected_label_ = new QLabel;
  selected_label_->setObjectName("cardTitle");
  lights_label_ = new QLabel;
  actions->addWidget(selected_label_);
  actions->addWidget(lights_label_);
  actions->addStretch();
  const QStringList key_names = {"↑ +10°", "↓ −10°", "L1 上一个", "R1 下一个"};
  for (int i = 0; i < 4; ++i) {
    key_labels_[i] = new QLabel(key_names[i]);
    key_labels_[i]->setStyleSheet("padding:6px 9px;color:#8290a6;background:#101723;border-radius:5px;");
    actions->addWidget(key_labels_[i]);
  }
  pad_layout->addLayout(actions);
  auto* manual = new QHBoxLayout;
  auto* previous = new QPushButton("← 电机");
  auto* next = new QPushButton("电机 →");
  step_up_ = new QPushButton("+10°");
  step_down_ = new QPushButton("−10°");
  step_up_->setObjectName("stepUpButton");
  step_down_->setObjectName("stepDownButton");
  step_up_->setEnabled(false);
  step_down_->setEnabled(false);
  manual->addWidget(previous);
  manual->addWidget(next);
  manual->addWidget(step_up_);
  manual->addWidget(step_down_);
  control_status_ = new QLabel;
  manual->addWidget(control_status_, 1);
  pad_layout->addLayout(manual);
  layout->addWidget(panel);
  connect(gamepad_refresh_, &QPushButton::clicked, this, [this] { RefreshGamepads(); });
  connect(gamepad_connect_, &QPushButton::clicked, this, [this] { ToggleGamepad(); });
  connect(previous, &QPushButton::clicked, this, [this] { SelectMotor(-1); });
  connect(next, &QPushButton::clicked, this, [this] { SelectMotor(1); });
  connect(step_up_, &QPushButton::clicked, this, [this] { StepMotor(10); });
  connect(step_down_, &QPushButton::clicked, this, [this] { StepMotor(-10); });
  auto* cards = new QHBoxLayout;
  cards->setSpacing(16);
  for (int i = 0; i < 3; ++i) {
    cards_[i] = new MotorCard(i + 1);
    cards->addWidget(cards_[i], 1);
  }
  layout->addLayout(cards, 1);
  auto* log_bar = new QHBoxLayout;
  counters_ = new QLabel("有效帧 0 · 无效行 0");
  auto* clear = new QPushButton("清空日志");
  log_bar->addWidget(counters_);
  log_bar->addStretch();
  log_bar->addWidget(clear);
  layout->addLayout(log_bar);
  log_ = new QPlainTextEdit;
  log_->setReadOnly(true);
  log_->setMaximumHeight(110);
  log_->document()->setMaximumBlockCount(500);
  log_->setPlaceholderText("连接 COM6 后显示原始回传；电流显示协议原始值，不是安培。灰色虚线为目标，绿色实线为实际角度。");
  layout->addWidget(log_);
  setStyleSheet(R"(
    QMainWindow,QWidget { background:#101723; color:#e8eef6; }
    QLabel#heading { font-size:25px; font-weight:700; }
    QLabel#muted { color:#8290a6; font-size:12px; }
    QFrame#motorCard { background:#172131; border:1px solid #2b384d; border-radius:12px; }
    QFrame#motorCard[selected="true"] { border:2px solid #4ce0bd; }
    QFrame#gamepadPanel { background:#172131; border:1px solid #34445c; border-radius:10px; }
    QFrame#gamepadPanel QLabel { background:transparent; }
    QFrame#motorCard QLabel,QFrame#motorCard QWidget { background:transparent; }
    QLabel#cardTitle { font-size:16px; font-weight:700; }
    QLabel#angle { font-size:38px; font-weight:700; color:#4ce0bd; }
    QLabel#metric { font-size:19px; font-weight:600; }
    QPushButton,QComboBox { padding:9px 14px; border:1px solid #34445c; border-radius:6px; background:#1a273a; }
    QPushButton:hover { background:#263a51; }
    QPushButton:disabled,QComboBox:disabled { color:#68778c; }
    QPushButton#connectButton { background:#227e6d; border-color:#34ac94; }
    QPlainTextEdit { background:#0c121c; border:1px solid #2b384d; border-radius:6px; padding:8px; font-family:Consolas; font-size:12px; }
  )");
  connect(refresh_button_, &QPushButton::clicked, this, [this] { RefreshPorts(); });
  connect(connect_button_, &QPushButton::clicked, this, [this] { ToggleConnection(); });
  connect(demo_button_, &QPushButton::clicked, this, [this] { ToggleDemo(); });
  connect(clear, &QPushButton::clicked, log_, &QPlainTextEdit::clear);
  connect(&serial_, &QSerialPort::readyRead, this, [this] { Receive(); });
  connect(&serial_, &QSerialPort::errorOccurred, this, [this](QSerialPort::SerialPortError error) {
    if (!closing_ && serial_.isOpen() && error != QSerialPort::NoError &&
        error != QSerialPort::TimeoutError) Disconnect("串口错误：" + serial_.errorString());
  });
  connect(&status_timer_, &QTimer::timeout, this, [this] { UpdateStatus(); });
  status_timer_.start(200);
  connect(&demo_timer_, &QTimer::timeout, this, [this] {
    ++demo_tick_;
    for (int i = 0; i < 3; ++i) {
      MotorFrame f;
      f.motor_id = i + 1; f.board_ms = demo_tick_ * 100; f.online = true;
      f.target_deg = demo_targets_[i];
      f.angle_deg = f.target_deg - std::sin(demo_tick_ * 0.08 + i) * 0.35;
      f.rpm = static_cast<int>(std::cos(demo_tick_ * 0.08 + i) * 15);
      f.command_current = f.rpm * 3; f.feedback_current = f.rpm * 2;
      f.temperature = 30 + i; f.encoder = (demo_tick_ * 10 + i * 1000) % 8192;
      UpdateFrame(f);
    }
  });
  gamepad_context_ = std::make_unique<dualforge::Context>();
  connect(&gamepad_timer_, &QTimer::timeout, this, [this] { PollGamepad(); });
  gamepad_timer_.setTimerType(Qt::PreciseTimer);
  gamepad_timer_.start(8);
  RefreshPorts();
  RefreshGamepads();
  UpdateSelection();
  if (start_demo) ToggleDemo();
}

void MainWindow::RefreshPorts() {
  const QString previous = ports_->currentData().toString();
  ports_->clear();
  for (const auto& port : QSerialPortInfo::availablePorts()) {
    ports_->addItem(port.portName() + " · " + port.description(), port.portName());
  }
  int index = ports_->findData(previous);
  if (index < 0) index = ports_->findData("COM6");
  if (index >= 0) ports_->setCurrentIndex(index);
  connect_button_->setEnabled(ports_->count() > 0 && !demo_);
}

void MainWindow::ToggleConnection() {
  if (serial_.isOpen()) { Disconnect("已断开"); return; }
  serial_.setPortName(ports_->currentData().toString());
  serial_.setBaudRate(QSerialPort::Baud115200);
  serial_.setDataBits(QSerialPort::Data8);
  serial_.setParity(QSerialPort::NoParity);
  serial_.setStopBits(QSerialPort::OneStop);
  serial_.setFlowControl(QSerialPort::NoFlowControl);
  if (!serial_.open(QIODevice::ReadWrite)) {
    connection_status_->setText("连接失败");
    AppendLog("连接失败：" + serial_.errorString() + "；请关闭占用串口的串口助手。");
    return;
  }
  serial_.setDataTerminalReady(true);
  parser_.Reset(); received_ = 0;
  arrivals_.fill(-1); online_.fill(false);
  for (auto* card : cards_) card->Reset();
  ports_->setEnabled(false); refresh_button_->setEnabled(false);
  demo_button_->setEnabled(false); connect_button_->setText("断开");
  connection_status_->setText("已连接 " + serial_.portName());
  AppendLog("连接成功，等待 C 板数据。");
}

void MainWindow::Disconnect(const QString& reason) {
  closing_ = true;
  serial_.close();
  closing_ = false;
  parser_.Reset();
  arrivals_.fill(-1); online_.fill(false);
  ports_->setEnabled(true); refresh_button_->setEnabled(true);
  demo_button_->setEnabled(true); connect_button_->setText("连接");
  connection_status_->setText(reason);
  AppendLog(reason);
  UpdateStatus();
}

void MainWindow::Receive() {
  QByteArray bytes = serial_.readAll();
  const auto frames = parser_.Feed(std::string_view(bytes.constData(), bytes.size()));
  for (const auto& frame : frames) UpdateFrame(frame);
  // Parser owns line assembly; raw chunks in this bounded log are diagnostic only.
  AppendLog(QString::fromLatin1(bytes).trimmed());
}

void MainWindow::UpdateFrame(const MotorFrame& frame) {
  ++received_;
  arrivals_[frame.motor_id - 1] = clock_.elapsed();
  online_[frame.motor_id - 1] = frame.online;
  cards_[frame.motor_id - 1]->SetFrame(frame, clock_.elapsed());
  UpdateStatus();
}

void MainWindow::UpdateStatus() {
  const bool ready = CanStep();
  step_up_->setEnabled(ready);
  step_down_->setEnabled(ready);
  control_status_->setText(demo_ ? "演示数据 · 点击 ±10° 仅修改模拟目标" :
      ready ? "按下一次移动 10° · 切换后其余电机保持目标" :
      "等待串口与当前电机的有效反馈");
  for (auto* card : cards_) card->UpdateAge(clock_.elapsed(), serial_.isOpen() || demo_);
  counters_->setText(QString("%1 · 有效帧 %2 · 无效行 %3")
      .arg(demo_ ? "演示数据" : "实时回传")
      .arg(static_cast<qulonglong>(received_))
      .arg(static_cast<qulonglong>(parser_.invalid_lines())));
}

void MainWindow::ToggleDemo() {
  demo_ = !demo_;
  parser_.Reset(); received_ = 0; demo_tick_ = 0;
  arrivals_.fill(-1); online_.fill(false); demo_targets_.fill(0);
  if (demo_ && gamepad_) CloseGamepad("演示模式已断开手柄");
  gamepads_->setEnabled(!demo_);
  gamepad_refresh_->setEnabled(!demo_);
  gamepad_connect_->setEnabled(!demo_ && gamepads_->count() > 0);
  for (auto* card : cards_) card->Reset();
  ports_->setEnabled(!demo_); refresh_button_->setEnabled(!demo_);
  connect_button_->setEnabled(!demo_ && ports_->count() > 0);
  demo_button_->setText(demo_ ? "退出演示" : "演示模式");
  connection_status_->setText(demo_ ? "演示模式 · 模拟数据" : "未连接");
  if (demo_) demo_timer_.start(100);
  else demo_timer_.stop();
  UpdateStatus();
}

void MainWindow::AppendLog(const QString& text) {
  if (!text.isEmpty()) log_->appendPlainText(text);
}

MainWindow::~MainWindow() {
  gamepad_timer_.stop();
  if (gamepad_) {
    gamepad_->set_player_lights(0);
    gamepad_->close();
    gamepad_.reset();
  }
  gamepad_context_.reset();
}

void MainWindow::RefreshGamepads() {
  if (gamepad_) return;
  const auto status = gamepad_context_->status();
  if (!status) {
    gamepad_status_->setText("驱动初始化失败");
    AppendLog(QString::fromStdString(status.message));
    gamepad_connect_->setEnabled(false);
    return;
  }
  const auto poll_status = gamepad_context_->poll();
  if (!poll_status) {
    gamepad_status_->setText("手柄刷新失败");
    AppendLog("手柄刷新失败：" + QString::fromStdString(poll_status.message));
    return;
  }
  const auto previous_id = gamepads_->currentData();
  gamepads_->clear();
  for (const auto& device : gamepad_context_->enumerate()) {
    // enumerate() leaves usb=false until open() can query the connection type.
    // Keep matching devices visible; the driver rejects wireless at open time.
    if (device.vendor == 0x054c && device.product == 0x0ce6) {
      gamepads_->addItem(QString::fromStdString(device.name),
                        static_cast<quint32>(device.id));
    }
  }
  const int previous_index = gamepads_->findData(previous_id);
  if (previous_index >= 0) gamepads_->setCurrentIndex(previous_index);
  gamepad_connect_->setEnabled(!demo_ && gamepads_->count() > 0);
  gamepad_status_->setText(gamepads_->count() ? "PS5 待连接" : "请用 USB 连接 PS5 后刷新");
  AppendLog(QString("发现 %1 个 DualSense；连接时检查 USB，请使用数据线。")
      .arg(gamepads_->count()));
  gamepad_timer_.start(8);
}

void MainWindow::ToggleGamepad() {
  if (gamepad_) { CloseGamepad("PS5 已断开"); return; }
  if (demo_ || gamepads_->currentIndex() < 0) return;
  auto result = gamepad_context_->open(gamepads_->currentData().toUInt());
  if (!result.status || !result.controller) {
    if (result.status.message == "v1 requires USB") {
      AppendLog("当前手柄使用蓝牙连接，请改用 USB 数据线后刷新并连接。");
      gamepad_status_->setText("需要 USB 数据线连接");
      return;
    }
    AppendLog("PS5 连接失败：" + QString::fromStdString(result.status.message));
    gamepad_status_->setText("PS5 连接失败，请刷新");
    return;
  }
  gamepad_ = std::move(result.controller);
  const auto buttons = gamepad_->state().buttons;
  motor_input_.SetBaseline({buttons.up, buttons.down, buttons.l1, buttons.r1});
  gamepads_->setEnabled(false);
  gamepad_refresh_->setEnabled(false);
  gamepad_connect_->setText("断开 PS5");
  gamepad_status_->setText("● PS5 USB 已连接");
  SetPlayerLights();
  AppendLog("PS5 已连接：↑ +10° / ↓ −10°；L1、R1 切换电机，长按不连发。");
}

void MainWindow::CloseGamepad(const QString& reason) {
  if (gamepad_) {
    if (gamepad_->is_connected()) gamepad_->set_player_lights(0);
    gamepad_->close();
    gamepad_.reset();
  }
  motor_input_.SetBaseline({});
  lights_ok_ = false;
  gamepads_->setEnabled(!demo_);
  gamepad_refresh_->setEnabled(!demo_);
  gamepad_connect_->setEnabled(!demo_ && gamepads_->count() > 0);
  gamepad_connect_->setText("连接 PS5");
  gamepad_status_->setText(reason);
  battery_->setText("电量 —");
  for (auto* label : key_labels_) {
    label->setStyleSheet("padding:6px 9px;color:#8290a6;background:#101723;border-radius:5px;");
  }
  UpdateSelection();
  AppendLog(reason + "；电机保留已有目标，不再接收手柄新指令。");
}

void MainWindow::PollGamepad() {
  const auto status = gamepad_context_->poll();
  if (!status) {
    if (gamepad_) CloseGamepad("PS5 驱动错误：" + QString::fromStdString(status.message));
    gamepad_timer_.stop();
    return;
  }
  if (!gamepad_) return;
  const auto state = gamepad_->state();
  if (!state.connected) { CloseGamepad("PS5 已拔出，请重新连接并刷新"); return; }
  battery_->setText(state.battery_percent >= 0 ?
      QString("电量 %1%").arg(state.battery_percent) : "电量未知");
  const auto& b = state.buttons;
  const bool pressed[] = {b.up, b.down, b.l1, b.r1};
  for (int i = 0; i < 4; ++i) {
    key_labels_[i]->setStyleSheet(pressed[i] ?
        "padding:6px 9px;color:#101723;background:#4ce0bd;border-radius:5px;" :
        "padding:6px 9px;color:#8290a6;background:#101723;border-radius:5px;");
  }
  // Track every sample even when stepping is unavailable. A held offline key
  // stays held after recovery, while a genuine new press fires immediately.
  const auto action = motor_input_.Update({b.up, b.down, b.l1, b.r1});
  if (action.selection_delta) SelectMotor(action.selection_delta);
  else if (action.step_degrees) StepMotor(action.step_degrees);
}

void MainWindow::SelectMotor(int direction) {
  selected_motor_ = NextMotor(selected_motor_, direction);
  if (gamepad_) SetPlayerLights();
  UpdateSelection();
  UpdateStatus();
}

void MainWindow::SetPlayerLights() {
  const auto status = gamepad_->set_player_lights(PlayerLightMask(selected_motor_));
  lights_ok_ = static_cast<bool>(status);
  if (!status) AppendLog("玩家灯设置失败：" + QString::fromStdString(status.message));
  UpdateSelection();
}

void MainWindow::UpdateSelection() {
  selected_label_->setText(QString("当前控制 M%1").arg(selected_motor_));
  QString pattern;
  const int mask = PlayerLightMask(selected_motor_);
  for (int i = 0; i < 5; ++i) pattern += mask & (1 << i) ? "● " : "○ ";
  lights_label_->setText(pattern + (gamepad_ ?
      (lights_ok_ ? " 已写入" : " 写入失败") : " 未连接"));
  for (int i = 0; i < 3; ++i) cards_[i]->Select(i + 1 == selected_motor_);
}

bool MainWindow::CanStep() const {
  const int index = selected_motor_ - 1;
  return (serial_.isOpen() || demo_) && online_[index] &&
      arrivals_[index] >= 0 && clock_.elapsed() - arrivals_[index] < 1000;
}

void MainWindow::StepMotor(int degrees) {
  if (degrees != 10 && degrees != -10) return;
  if (!CanStep()) {
    AppendLog(QString("M%1 未收到有效反馈，已忽略 %2° 指令。").arg(selected_motor_).arg(degrees));
    return;
  }
  if (demo_) {
    demo_targets_[selected_motor_ - 1] += degrees;
    AppendLog(QString("演示 M%1：目标 %2°").arg(selected_motor_)
        .arg(demo_targets_[selected_motor_ - 1]));
    return;
  }
  // Do not queue delayed motion when the USB link stops draining.
  if (serial_.bytesToWrite() > 128) {
    AppendLog("串口发送积压，本次角度指令已忽略。");
    return;
  }
  const QByteArray command = QString("STEP %1 %2\n").arg(selected_motor_)
      .arg(degrees > 0 ? "+10" : "-10").toLatin1();
  if (serial_.write(command) != command.size()) {
    Disconnect("角度指令发送失败：" + serial_.errorString());
    return;
  }
  AppendLog("发送 " + QString::fromLatin1(command).trimmed() +
            "；目标角度以 C 板后续回传为准。");
}
