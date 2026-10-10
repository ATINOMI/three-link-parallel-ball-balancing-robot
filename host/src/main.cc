#include <QApplication>
#include <QFont>
#include <QTimer>
#include <QPixmap>
#include <QFontDatabase>
#include "main_window.h"

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  app.setApplicationName("MotorMonitor");
  if (app.arguments().contains("--screenshot")) {
    QFontDatabase::addApplicationFont("C:/Windows/Fonts/msyh.ttc");
  }
  app.setFont(QFont("Microsoft YaHei", 10));
  MainWindow window(app.arguments().contains("--demo"));
  if (app.arguments().contains("--screenshot")) window.resize(1280, 1020);
  window.show();
  const int screenshot_option = app.arguments().indexOf("--screenshot");
  if (screenshot_option >= 0 && screenshot_option + 1 < app.arguments().size()) {
    const QString path = app.arguments()[screenshot_option + 1];
    QTimer::singleShot(1800, &app, [&window, &app, path] {
      app.exit(window.grab().save(path) ? 0 : 1);
    });
  }
  return app.exec();
}
