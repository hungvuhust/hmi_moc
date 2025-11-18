#include "rtcrobot_hmi_denso/RobotData.hpp"
#include <QDebug>
#include <QProcess>
#include <QRegExp>

RobotData::RobotData(QObject* parent) : QObject(parent) {
  // Initialize default values
  m_mapList = QStringList() << "Map_01"
                            << "Map_02"
                            << "Map_03"
                            << "Map_04";
  m_currentMapName = "Map_01";

  // Initialize log arrays
  m_logDateArray = QStringList() << "2025-01-01"
                                 << "2025-01-02"
                                 << "2025-01-03";
  m_logTimeArray = QStringList() << "10:00:00"
                                 << "11:00:00"
                                 << "12:00:00";
  m_logMessageArray = QStringList() << "Log message 1"
                                    << "Log message 2"
                                    << "Log message 3";

  setEmgFrontError(true);
}

// Navigation setters
void RobotData::setPosX(double value) {
  if (qFuzzyCompare(m_posX, value))
    return;
  m_posX = value;
  emit posXChanged();
}

void RobotData::setPosY(double value) {
  if (qFuzzyCompare(m_posY, value))
    return;
  m_posY = value;
  emit posYChanged();
}

void RobotData::setTheta(double value) {
  if (qFuzzyCompare(m_theta, value))
    return;
  m_theta = value;
  emit thetaChanged();
}

void RobotData::setSpeed(double value) {
  if (qFuzzyCompare(m_speed, value))
    return;
  m_speed = value;
  emit speedChanged();
}

void RobotData::setMapName(const QString& value) {
  if (m_mapName == value)
    return;
  m_mapName = value;
  emit mapNameChanged();
}

void RobotData::setDestination(const QString& value) {
  if (m_destination == value)
    return;
  m_destination = value;
  emit destinationChanged();
}

void RobotData::setLocalizationScore(double value) {
  if (qFuzzyCompare(m_localizationScore, value))
    return;
  m_localizationScore = value;
  emit localizationScoreChanged();
}

// Status setters
void RobotData::setBatteryLevel(double value) {
  if (qFuzzyCompare(m_batteryLevel, value))
    return;
  m_batteryLevel = value;
  emit batteryLevelChanged();
}

void RobotData::setRobotState(const QString& value) {
  if (m_robotState == value)
    return;
  m_robotState = value;
  emit robotStateChanged();
}

void RobotData::setOperationMode(const QString& value) {
  if (m_operationMode == value)
    return;
  m_operationMode = value;
  emit operationModeChanged();
}

// Device error setters
void RobotData::setEmgFrontError(bool value) {
  if (m_emgFrontError == value)
    return;
  m_emgFrontError = value;
  emit emgFrontErrorChanged();
}

void RobotData::setEmgRearError(bool value) {
  if (m_emgRearError == value)
    return;
  m_emgRearError = value;
  emit emgRearErrorChanged();
}

void RobotData::setMotorLeftError(bool value) {
  if (m_motorLeftError == value)
    return;
  m_motorLeftError = value;
  emit motorLeftErrorChanged();
}

void RobotData::setMotorRightError(bool value) {
  if (m_motorRightError == value)
    return;
  m_motorRightError = value;
  emit motorRightErrorChanged();
}

void RobotData::setLidarFrontError(bool value) {
  if (m_lidarFrontError == value)
    return;
  m_lidarFrontError = value;
  emit lidarFrontErrorChanged();
}

void RobotData::setLidarRearError(bool value) {
  if (m_lidarRearError == value)
    return;
  m_lidarRearError = value;
  emit lidarRearErrorChanged();
}

// Map Panel setters
void RobotData::setMapList(const QStringList& value) {
  if (m_mapList == value)
    return;
  m_mapList = value;
  emit mapListChanged();
}

void RobotData::setCurrentMapName(const QString& value) {
  if (m_currentMapName == value)
    return;
  m_currentMapName = value;
  emit currentMapNameChanged();
}

void RobotData::setOriginX(double value) {
  if (qFuzzyCompare(m_originX, value))
    return;
  m_originX = value;
  emit originXChanged();
}

void RobotData::setOriginY(double value) {
  if (qFuzzyCompare(m_originY, value))
    return;
  m_originY = value;
  emit originYChanged();
}

void RobotData::setOriginTheta(double value) {
  if (qFuzzyCompare(m_originTheta, value))
    return;
  m_originTheta = value;
  emit originThetaChanged();
}

// Info Page setters
void RobotData::setLogDateArray(const QStringList& value) {
  if (m_logDateArray == value)
    return;
  m_logDateArray = value;
  emit logDateArrayChanged();
}

void RobotData::setLogTimeArray(const QStringList& value) {
  if (m_logTimeArray == value)
    return;
  m_logTimeArray = value;
  emit logTimeArrayChanged();
}

void RobotData::setLogMessageArray(const QStringList& value) {
  if (m_logMessageArray == value)
    return;
  m_logMessageArray = value;
  emit logMessageArrayChanged();
}

void RobotData::setMaxVelocityInternal(double value) {
  if (qFuzzyCompare(m_maxVelocity, value))
    return;
  m_maxVelocity = value;
  emit maxVelocityChanged();
}

// Q_INVOKABLE methods
void RobotData::setSpeed(double linear, double angular) {
  qDebug() << "RobotData::setSpeed - Linear:" << linear
           << "Angular:" << angular;
  // TODO: Implement ROS 2 command to set robot speed
  // Example: publish to /cmd_vel topic
}

void RobotData::setMaxVelocity(double velocity) {
  qDebug() << "RobotData::setMaxVelocity - Max Velocity:" << velocity;
  setMaxVelocityInternal(velocity);
  // TODO: Implement max velocity setting in robot control
}

void RobotData::start() {
  qDebug() << "RobotData::start";
  setRobotState("RUNNING");
  // TODO: Implement start command
}

void RobotData::stop() {
  qDebug() << "RobotData::stop";
  setRobotState("IDLE");
  // TODO: Implement stop command
}

void RobotData::relocation() {
  qDebug() << "RobotData::relocation";
  setRobotState("RELOCATING");
  // TODO: Implement relocation command
}

void RobotData::selectMap(const QString& mapName) {
  qDebug() << "RobotData::selectMap - Map:" << mapName;
  setCurrentMapName(mapName);
  // TODO: Implement map selection
}

void RobotData::addTag() {
  qDebug() << "RobotData::addTag";
  // TODO: Implement add tag functionality
}

void RobotData::reset() {
  qDebug() << "RobotData::reset";
  // TODO: Implement reset functionality
}

void RobotData::clearLog() {
  qDebug() << "RobotData::clearLog";
  setLogDateArray(QStringList());
  setLogTimeArray(QStringList());
  setLogMessageArray(QStringList());
  // TODO: Implement clear log functionality
}
