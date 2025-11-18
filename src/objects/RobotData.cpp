#include "rtcrobot_hmi_denso/RobotData.hpp"
#include <qobject.h>
#include <QDebug>
#include <QProcess>
#include <QRegExp>
#include <QThread>

RobotData::RobotData(QObject* parent) : QObject(parent) {
  initDatabase();
  getAllMaps();

  std::thread thread{[this]() { spinThread(); }};
  thread.detach();

  m_updateStateDataTimer = std::make_shared<QTimer>(this);
  connect(m_updateStateDataTimer.get(),
          &QTimer::timeout,
          this,
          &RobotData::updateStateData);
  m_updateStateDataTimer->start(500);
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
  std::lock_guard<std::mutex> lock(m_Mutex);
  // Publish cmd_vel through RosClient
  if (m_rosClient) {
    m_rosClient->publishCmdVel(linear, angular);
  } else {
    qDebug() << "RosClient not initialized";
  }
}

void RobotData::setMaxVelocity(double velocity) {
  qDebug() << "RobotData::setMaxVelocity - Max Velocity:" << velocity;
  setMaxVelocityInternal(velocity);
  // TODO: Implement max velocity setting in robot control
}

void RobotData::start() {
  qDebug() << "RobotData::start";

  if (m_rosClient) {
    {
      std::lock_guard<std::mutex> lock(m_Mutex);
      m_rosClient->setStart();
    }
    // Wait for service response then show notification
    QTimer::singleShot(500, [this]() {
      std::string msg = m_rosClient->getReturnMessageOfService();
      if (!msg.empty()) {
        bool isSuccess = msg.find("success") != std::string::npos ||
                         msg.find("Started") != std::string::npos;
        emit showNotification(QString::fromStdString(msg),
                              isSuccess ? "success" : "error");
      }
    });
  } else {
    qDebug() << "RosClient not initialized";
    emit showNotification("RosClient not initialized", "error");
  }
}

void RobotData::stop() {
  qDebug() << "RobotData::stop";

  // TODO: Implement stop command
  if (m_rosClient) {
    std::lock_guard<std::mutex> lock(m_Mutex);
    m_rosClient->setStop();
  } else {
    qDebug() << "RosClient not initialized";
  }
}

void RobotData::relocation() {
  qDebug() << "RobotData::relocation";

  if (m_rosClient) {
    {
      std::lock_guard<std::mutex> lock(m_Mutex);
      m_rosClient->setRelocation();
    }
    // Wait for service response then show notification
    QTimer::singleShot(500, [this]() {
      std::string msg = m_rosClient->getReturnMessageOfService();
      if (!msg.empty()) {
        bool isSuccess = msg.find("success") != std::string::npos ||
                         msg.find("Successfully") != std::string::npos;
        emit showNotification(QString::fromStdString(msg),
                              isSuccess ? "success" : "error");
      }
    });
  } else {
    qDebug() << "RosClient not initialized";
    emit showNotification("RosClient not initialized", "error");
  }
}

void RobotData::selectMap(const QString& mapName) {
  qDebug() << "RobotData::selectMap - Map:" << mapName;

  if (m_rosClient) {
    {
      std::lock_guard<std::mutex> lock(m_Mutex);
      m_rosClient->setSelectMap(mapName.toStdString());
    }
    // Wait for service response then show notification
    QTimer::singleShot(500, [this]() {
      std::string msg = m_rosClient->getReturnMessageOfService();
      if (!msg.empty()) {
        bool isSuccess = msg.find("success") != std::string::npos ||
                         msg.find("Loaded") != std::string::npos;
        emit showNotification(QString::fromStdString(msg),
                              isSuccess ? "success" : "error");
      }
    });
  } else {
    qDebug() << "RosClient not initialized";
    emit showNotification("RosClient not initialized", "error");
  }
}

void RobotData::addTag() {
  qDebug() << "RobotData::addTag";
  if (m_rosClient) {
    m_rosClient->setAddTag();
    // Wait a bit for service response
    QTimer::singleShot(500, [this]() {
      std::string msg = m_rosClient->getReturnMessageOfService();
      if (!msg.empty()) {
        bool isSuccess = msg.find("success") != std::string::npos ||
                         msg.find("Added") != std::string::npos;
        emit showNotification(QString::fromStdString(msg),
                              isSuccess ? "success" : "error");
      }
    });
  }
}

void RobotData::reset() {
  qDebug() << "RobotData::reset";
  // TODO: Implement reset functionality
  if (m_rosClient) {
    std::lock_guard<std::mutex> lock(m_Mutex);
    m_rosClient->setReset();
  } else {
    qDebug() << "RosClient not initialized";
  }
}

void RobotData::clearLog() {
  qDebug() << "RobotData::clearLog";
  setLogDateArray(QStringList());
  setLogTimeArray(QStringList());
  setLogMessageArray(QStringList());
  // TODO: Implement clear log functionality
}

void RobotData::onPageChanged(int pageIndex, const QString& pageName) {
  qDebug() << "RobotData::onPageChanged - Page:" << pageIndex << pageName;

  switch (pageIndex) {
    case 0:  // HOME
      qDebug() << "Home page opened";
      // TODO: Handle home page logic
      break;
    case 1:  // MANUAL
      qDebug() << "Manual page opened";
      // TODO: Handle manual page logic
      break;
    case 2:  // SETTINGS
      qDebug() << "Settings page opened";
      // TODO: Handle settings page logic
      break;
    case 3:  // MAPS
      qDebug() << "Maps page opened";
      // TODO: Handle maps page logic - Load danh sách maps, refresh data, etc.
      // Load map list
      this->getAllMaps();
      this->setMapList(m_mapList);

      break;
    case 4:  // INFO
      qDebug() << "Info page opened";
      // TODO: Handle info page logic
      break;
    default:
      qDebug() << "Unknown page:" << pageIndex;
      break;
  }
}

int RobotData::spinThread() {
  rclcpp::init(0, nullptr);
  auto node            = std::make_shared<rclcpp::Node>("robot_data");
  m_rosClient          = std::make_shared<RosClient>();
  auto& signal_handler = rtcrobot_core::SignalHandler::getInstance();
  int   exit_code      = signal_handler.spinWithSignalHandling(m_rosClient);
  rclcpp::shutdown();
  return exit_code;
}

void RobotData::updateStateData() {
  if (m_rosClient) {
    std::lock_guard<std::mutex> lock(m_Mutex);
    vda5050_msgs::msg::State    state;
    if (m_rosClient->getLatestState(state)) {
      this->setPosX(state.agv_position.x);
      this->setPosY(state.agv_position.y);
      this->setTheta(state.agv_position.theta);
      this->setSpeed(state.velocity.vx);
      this->setMapName(QString::fromStdString(state.agv_position.map_id));
      // m_destination       = state.destination;
      this->setLocalizationScore(state.agv_position.localization_score);
      this->setBatteryLevel(state.battery_state.battery_charge);
      this->setOperationMode(QString::fromStdString(state.operating_mode));

      this->setRobotState(
        QString::fromStdString(m_rosClient->getStateAGV().parent_state.empty()
                                 ? m_rosClient->getStateAGV().state
                                 : m_rosClient->getStateAGV().parent_state));
    }
  }
}

void RobotData::initDatabase() {
  // Khởi tạo Drogon database client
  try {
    // Đọc thông tin database từ model.json
    Json::Value   config;
    std::ifstream config_file(
      ament_index_cpp::get_package_share_directory("rtcrobot_hmi_denso") +
      "/models/"
      "model.json");
    if (config_file.is_open()) {
      config_file >> config;
      config_file.close();

      // Tạo connection string cho Drogon
      std::string host   = config["host"].asString();
      int         port   = config["port"].asInt();
      std::string dbname = config["dbname"].asString();
      std::string user   = config["user"].asString();
      std::string passwd = config["passwd"].asString();

      std::string conn_str = "host=" + host + " port=" + std::to_string(port) +
                             " dbname=" + dbname + " user=" + user +
                             " password=" + passwd;

      // Tạo database client
      m_dbClient = drogon::orm::DbClient::newPgClient(conn_str, 1);

      qDebug() << "Successfully connected to Drogon ORM database";
    } else {
      qDebug() << "Cannot read config file models/model.json";
    }
  } catch (const std::exception& e) {
    qDebug() << "Error initializing database: " << e.what();
    throw;
  }
}

void RobotData::getAllMaps() {
  if (m_dbClient) {
    drogon::orm::Mapper<drogon_model::robotdb::RobotMaps> mapper(m_dbClient);
    auto maps = mapper.findAll();
    for (const auto& map : maps) {
      qDebug() << "Map:" << QString::fromStdString(map.getValueOfName());
      m_mapList.append(QString::fromStdString(map.getValueOfName()));
    }
  }
}