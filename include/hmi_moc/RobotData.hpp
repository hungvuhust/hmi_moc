#ifndef ROBOT_DATA_HPP
#define ROBOT_DATA_HPP

#include <QObject>
#include <QString>
#include <QStringList>

class RobotData : public QObject {
  Q_OBJECT

  // Navigation properties
  Q_PROPERTY(double posX READ posX NOTIFY posXChanged)
  Q_PROPERTY(double posY READ posY NOTIFY posYChanged)
  Q_PROPERTY(double theta READ theta NOTIFY thetaChanged)
  Q_PROPERTY(double speed READ speed NOTIFY speedChanged)
  Q_PROPERTY(QString mapName READ mapName NOTIFY mapNameChanged)
  Q_PROPERTY(QString destination READ destination NOTIFY destinationChanged)
  Q_PROPERTY(double localizationScore READ localizationScore NOTIFY
               localizationScoreChanged)

  // Status properties
  Q_PROPERTY(double batteryLevel READ batteryLevel NOTIFY batteryLevelChanged)
  Q_PROPERTY(QString robotState READ robotState NOTIFY robotStateChanged)
  Q_PROPERTY(
    QString operationMode READ operationMode NOTIFY operationModeChanged)

  // Settings properties

  Q_PROPERTY(double maxVelocity READ maxVelocity WRITE setMaxVelocityInternal
               NOTIFY maxVelocityChanged)

  // Device error properties
  Q_PROPERTY(bool emgFrontError READ emgFrontError NOTIFY emgFrontErrorChanged)
  Q_PROPERTY(bool emgRearError READ emgRearError NOTIFY emgRearErrorChanged)
  Q_PROPERTY(
    bool motorLeftError READ motorLeftError NOTIFY motorLeftErrorChanged)
  Q_PROPERTY(
    bool motorRightError READ motorRightError NOTIFY motorRightErrorChanged)
  Q_PROPERTY(
    bool lidarFrontError READ lidarFrontError NOTIFY lidarFrontErrorChanged)
  Q_PROPERTY(
    bool lidarRearError READ lidarRearError NOTIFY lidarRearErrorChanged)

  // Map Panel properties
  Q_PROPERTY(QStringList mapList READ mapList NOTIFY mapListChanged)
  Q_PROPERTY(
    QString currentMapName READ currentMapName NOTIFY currentMapNameChanged)
  Q_PROPERTY(double originX READ originX NOTIFY originXChanged)
  Q_PROPERTY(double originY READ originY NOTIFY originYChanged)
  Q_PROPERTY(double originTheta READ originTheta NOTIFY originThetaChanged)

  // Info Page properties
  Q_PROPERTY(
    QStringList logDateArray READ logDateArray NOTIFY logDateArrayChanged)
  Q_PROPERTY(
    QStringList logTimeArray READ logTimeArray NOTIFY logTimeArrayChanged)
  Q_PROPERTY(QStringList logMessageArray READ logMessageArray NOTIFY
               logMessageArrayChanged)

public:
  explicit RobotData(QObject* parent = nullptr);

  // Navigation getters
  double posX() const {
    return m_posX;
  }
  double posY() const {
    return m_posY;
  }
  double theta() const {
    return m_theta;
  }
  double speed() const {
    return m_speed;
  }
  QString mapName() const {
    return m_mapName;
  }
  QString destination() const {
    return m_destination;
  }
  double localizationScore() const {
    return m_localizationScore;
  }

  // Status getters
  double batteryLevel() const {
    return m_batteryLevel;
  }
  QString robotState() const {
    return m_robotState;
  }
  QString operationMode() const {
    return m_operationMode;
  }

  // Settings getters

  double maxVelocity() const {
    return m_maxVelocity;
  }

  // Device error getters
  bool emgFrontError() const {
    return m_emgFrontError;
  }
  bool emgRearError() const {
    return m_emgRearError;
  }
  bool motorLeftError() const {
    return m_motorLeftError;
  }
  bool motorRightError() const {
    return m_motorRightError;
  }
  bool lidarFrontError() const {
    return m_lidarFrontError;
  }
  bool lidarRearError() const {
    return m_lidarRearError;
  }

  // Map Panel getters
  QStringList mapList() const {
    return m_mapList;
  }
  QString currentMapName() const {
    return m_currentMapName;
  }
  double originX() const {
    return m_originX;
  }
  double originY() const {
    return m_originY;
  }
  double originTheta() const {
    return m_originTheta;
  }

  // Info Page getters
  QStringList logDateArray() const {
    return m_logDateArray;
  }
  QStringList logTimeArray() const {
    return m_logTimeArray;
  }
  QStringList logMessageArray() const {
    return m_logMessageArray;
  }

  // Setters
  void setPosX(double value);
  void setPosY(double value);
  void setTheta(double value);
  void setSpeed(double value);
  void setMapName(const QString& value);
  void setDestination(const QString& value);
  void setLocalizationScore(double value);
  void setBatteryLevel(double value);
  void setRobotState(const QString& value);
  void setOperationMode(const QString& value);
  void setMaxVelocityInternal(double value);
  void setEmgFrontError(bool value);
  void setEmgRearError(bool value);
  void setMotorLeftError(bool value);
  void setMotorRightError(bool value);
  void setLidarFrontError(bool value);
  void setLidarRearError(bool value);
  void setMapList(const QStringList& value);
  void setCurrentMapName(const QString& value);
  void setOriginX(double value);
  void setOriginY(double value);
  void setOriginTheta(double value);
  void setLogDateArray(const QStringList& value);
  void setLogTimeArray(const QStringList& value);
  void setLogMessageArray(const QStringList& value);

  // Q_INVOKABLE methods for QML
  Q_INVOKABLE void setSpeed(double linear, double angular);
  Q_INVOKABLE void setMaxVelocity(double velocity);
  Q_INVOKABLE void start();
  Q_INVOKABLE void stop();
  Q_INVOKABLE void relocation();
  Q_INVOKABLE void selectMap(const QString& mapName);
  Q_INVOKABLE void addTag();
  Q_INVOKABLE void reset();
  Q_INVOKABLE void clearLog();

signals:
  // Navigation signals
  void posXChanged();
  void posYChanged();
  void thetaChanged();
  void speedChanged();
  void mapNameChanged();
  void destinationChanged();
  void localizationScoreChanged();

  // Status signals
  void batteryLevelChanged();
  void robotStateChanged();
  void operationModeChanged();

  // Settings signals

  void maxVelocityChanged();

  // Device error signals
  void emgFrontErrorChanged();
  void emgRearErrorChanged();
  void motorLeftErrorChanged();
  void motorRightErrorChanged();
  void lidarFrontErrorChanged();
  void lidarRearErrorChanged();

  // Map Panel signals
  void mapListChanged();
  void currentMapNameChanged();
  void originXChanged();
  void originYChanged();
  void originThetaChanged();

  // Info Page signals
  void logDateArrayChanged();
  void logTimeArrayChanged();
  void logMessageArrayChanged();

private:
  // Navigation data
  double  m_posX              = 0.0;
  double  m_posY              = 0.0;
  double  m_theta             = 0.0;
  double  m_speed             = 0.0;
  QString m_mapName           = "Map current";
  QString m_destination       = "None";
  double  m_localizationScore = 0.0;

  // Status data
  double  m_batteryLevel  = 80.0;
  QString m_robotState    = "IDLE";
  QString m_operationMode = "AUTO";

  // Settings data
  double m_maxVelocity = 0.5;

  // Device error flags
  bool m_emgFrontError   = false;
  bool m_emgRearError    = false;
  bool m_motorLeftError  = false;
  bool m_motorRightError = false;
  bool m_lidarFrontError = false;
  bool m_lidarRearError  = false;

  // Map Panel data
  QStringList m_mapList;
  QString     m_currentMapName;
  double      m_originX     = 0.0;
  double      m_originY     = 0.0;
  double      m_originTheta = 0.0;

  // Info Page data
  QStringList m_logDateArray;
  QStringList m_logTimeArray;
  QStringList m_logMessageArray;
};

#endif  // ROBOT_DATA_HPP
