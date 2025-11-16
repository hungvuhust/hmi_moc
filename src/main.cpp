#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include "hmi_moc/RobotData.hpp"
#include "hmi_moc/DeviceStatus.hpp"

int main(int argc, char* argv[]) {
  QGuiApplication       app(argc, argv);
  QQmlApplicationEngine engine;

  // Create instances
  RobotData*    robotData    = new RobotData(&app);
  DeviceStatus* deviceStatus = new DeviceStatus(&app);

  // Register types for QML
  qmlRegisterType<RobotData>("HmiMoc", 1, 0, "RobotData");
  qmlRegisterType<DeviceStatus>("HmiMoc", 1, 0, "DeviceStatus");

  // Expose to QML as global objects
  engine.rootContext()->setContextProperty("robotData", robotData);
  engine.rootContext()->setContextProperty("deviceStatus", deviceStatus);

  const QUrl url(QStringLiteral("qrc:/qml/hmi.qml"));
  engine.load(url);

  return app.exec();
}
