#ifndef DEVICE_STATUS_HPP
#define DEVICE_STATUS_HPP

#include <QObject>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QNetworkInterface>

class DeviceStatus : public QObject {
  Q_OBJECT

  Q_PROPERTY(QString cpuUsage READ cpuUsage NOTIFY cpuUsageChanged)
  Q_PROPERTY(QString ramUsage READ ramUsage NOTIFY ramUsageChanged)
  Q_PROPERTY(QString temperature READ temperature NOTIFY temperatureChanged)
  Q_PROPERTY(QString memUsage READ memUsage NOTIFY memUsageChanged)
  Q_PROPERTY(QString timeUsed READ timeUsed NOTIFY timeUsedChanged)
  Q_PROPERTY(
    double volume READ volume WRITE setVolumeInternal NOTIFY volumeChanged)
  Q_PROPERTY(
    QStringList lanIpAddresses READ lanIpAddresses NOTIFY lanIpAddressesChanged)

public:
  explicit DeviceStatus(QObject* parent = nullptr);
  ~DeviceStatus();

  QString cpuUsage() const {
    return m_cpuUsage;
  }
  QString ramUsage() const {
    return m_ramUsage;
  }
  QString temperature() const {
    return m_temperature;
  }
  QString memUsage() const {
    return m_memUsage;
  }
  QString timeUsed() const {
    return m_timeUsed;
  }
  double volume() const {
    return m_volume;
  }
  QStringList lanIpAddresses() const {
    return m_lanIpAddresses;
  }

  void setCpuUsage(const QString& value);
  void setRamUsage(const QString& value);
  void setTemperature(const QString& value);
  void setMemUsage(const QString& value);
  void setTimeUsed(const QString& value);
  void setLanIpAddresses(const QStringList& value);

  void             setVolumeInternal(double value);
  Q_INVOKABLE void setVolume(double volume);
  Q_INVOKABLE void refreshNetworkInfo();

signals:
  void cpuUsageChanged();
  void ramUsageChanged();
  void temperatureChanged();
  void memUsageChanged();
  void timeUsedChanged();
  void volumeChanged();
  void lanIpAddressesChanged();

private slots:
  void updateSystemData();

private:
  void readCpuUsage();
  void readRamUsage();
  void readTemperature();
  void readMemUsage();
  void readTimeUsed();
  void readLanIpAddresses();

  QString     m_cpuUsage    = "0%";
  QString     m_ramUsage    = "0%";
  QString     m_temperature = "0°C";
  QString     m_memUsage    = "0%";
  QString     m_timeUsed    = "00:00:00";
  QStringList m_lanIpAddresses;
  double      m_volume      = 50.0;
  QTimer*     m_updateTimer = nullptr;

  // For CPU calculation
  qint64 m_lastCpuTotal = 0;
  qint64 m_lastCpuIdle  = 0;
  qint64 m_startTime    = 0;

  // Helper functions
  void readSystemVolume();
};

#endif  // DEVICE_STATUS_HPP
