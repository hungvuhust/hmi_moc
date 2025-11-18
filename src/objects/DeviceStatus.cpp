#include "rtcrobot_hmi_denso/DeviceStatus.hpp"
#include <QFile>
#include <QTextStream>
#include <QProcess>
#include <QDateTime>
#include <QDebug>
#include <QElapsedTimer>
#include <QRegExp>
#include <ctime>

DeviceStatus::DeviceStatus(QObject* parent)
  : QObject(parent), m_startTime(QDateTime::currentMSecsSinceEpoch()) {
  // Create and setup timer
  m_updateTimer = new QTimer(this);
  connect(m_updateTimer,
          &QTimer::timeout,
          this,
          &DeviceStatus::updateSystemData);
  m_updateTimer->start(1000);  // Update every 1 second

  // Initial update
  updateSystemData();

  // Read system volume on startup
  readSystemVolume();
}

DeviceStatus::~DeviceStatus() {
  if (m_updateTimer) {
    m_updateTimer->stop();
  }
}

void DeviceStatus::updateSystemData() {
  readCpuUsage();
  readRamUsage();
  readTemperature();
  readMemUsage();
  readTimeUsed();
}

void DeviceStatus::readCpuUsage() {
  QFile file("/proc/stat");
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    return;

  QTextStream in(&file);
  QString     line = in.readLine();
  file.close();

  if (!line.startsWith("cpu "))
    return;

  QStringList parts = line.split(QRegExp("\\s+"), Qt::SkipEmptyParts);
  if (parts.size() < 8)
    return;

  qint64 user    = parts[1].toLongLong();
  qint64 nice    = parts[2].toLongLong();
  qint64 system  = parts[3].toLongLong();
  qint64 idle    = parts[4].toLongLong();
  qint64 iowait  = parts[5].toLongLong();
  qint64 irq     = parts[6].toLongLong();
  qint64 softirq = parts[7].toLongLong();

  qint64 total     = user + nice + system + idle + iowait + irq + softirq;
  qint64 totalIdle = idle + iowait;

  if (m_lastCpuTotal != 0) {
    qint64 totalDiff = total - m_lastCpuTotal;
    qint64 idleDiff  = totalIdle - m_lastCpuIdle;

    if (totalDiff > 0) {
      double  cpuPercent = 100.0 * (totalDiff - idleDiff) / totalDiff;
      QString cpuStr     = QString::number(cpuPercent, 'f', 1) + "%";
      setCpuUsage(cpuStr);
    }
  }

  m_lastCpuTotal = total;
  m_lastCpuIdle  = totalIdle;
}

void DeviceStatus::readRamUsage() {
  QFile file("/proc/meminfo");
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    return;

  qint64 totalMem     = 0;
  qint64 freeMem      = 0;
  qint64 availableMem = 0;

  // Read all content at once
  QByteArray data = file.readAll();
  file.close();

  QString     content = QString::fromUtf8(data);
  QStringList lines   = content.split('\n');

  for (const QString& line : lines) {
    if (line.isEmpty())
      continue;

    int colonPos = line.indexOf(':');
    if (colonPos == -1)
      continue;

    QString key   = line.left(colonPos).trimmed();
    QString value = line.mid(colonPos + 1).trimmed();

    // Remove "kB" suffix if present and extract number
    value = value.split(' ').first();

    bool   ok;
    qint64 numValue = value.toLongLong(&ok);
    if (!ok)
      continue;

    if (key == "MemTotal") {
      totalMem = numValue;
    } else if (key == "MemFree") {
      freeMem = numValue;
    } else if (key == "MemAvailable") {
      availableMem = numValue;
    }
  }

  if (totalMem > 0) {
    qint64  usedMem    = totalMem - (availableMem > 0 ? availableMem : freeMem);
    double  ramPercent = 100.0 * usedMem / totalMem;
    QString ramStr     = QString::number(ramPercent, 'f', 1) + "%";
    setRamUsage(ramStr);
  }
}

void DeviceStatus::readTemperature() {
  // Try common temperature sensor paths
  QStringList tempPaths = {"/sys/class/thermal/thermal_0/temp",
                           "/sys/class/thermal/thermal_1/temp",
                           "/sys/class/hwmon/hwmon0/temp1_input",
                           "/sys/class/hwmon/hwmon1/temp1_input"};

  for (const QString& path : tempPaths) {
    QFile file(path);
    if (file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      QTextStream in(&file);
      QString     line = in.readLine().trimmed();
      file.close();

      bool ok;
      int  tempMilliC = line.toInt(&ok);
      if (ok && tempMilliC > 0) {
        double  tempC   = tempMilliC / 1000.0;
        QString tempStr = QString::number(tempC, 'f', 1) + "°C";
        setTemperature(tempStr);
        return;
      }
    }
  }

  // Fallback: try sensors command
  QProcess process;
  process.start("sensors", QStringList(), QIODevice::ReadOnly);
  if (process.waitForFinished(500)) {
    QByteArray output    = process.readAllStandardOutput();
    QString    outputStr = QString::fromUtf8(output);
    // Parse temperature from sensors output (simplified)
    QRegExp    rx("([0-9]+\\.[0-9]+)°C");
    if (rx.indexIn(outputStr) != -1) {
      QString tempStr = rx.cap(1) + "°C";
      setTemperature(tempStr);
      return;
    }
  }
}

void DeviceStatus::readMemUsage() {
  QFile file("/proc/meminfo");
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
    qDebug() << "Failed to open /proc/meminfo";
    return;
  }

  qint64 totalMem = 0;
  qint64 freeMem  = 0;
  qint64 buffers  = 0;
  qint64 cached   = 0;

  // Read all content at once
  QByteArray data = file.readAll();
  file.close();

  QString     content = QString::fromUtf8(data);
  QStringList lines   = content.split('\n');

  for (const QString& line : lines) {
    if (line.isEmpty())
      continue;

    int colonPos = line.indexOf(':');
    if (colonPos == -1)
      continue;

    QString key   = line.left(colonPos).trimmed();
    QString value = line.mid(colonPos + 1).trimmed();

    // Remove "kB" suffix if present and extract number
    value = value.split(' ').first();

    bool   ok;
    qint64 numValue = value.toLongLong(&ok);
    if (!ok)
      continue;

    if (key == "MemTotal") {
      totalMem = numValue;
    } else if (key == "MemFree") {
      freeMem = numValue;
    } else if (key == "Buffers") {
      buffers = numValue;
    } else if (key == "Cached") {
      cached = numValue;
    }
  }

  if (totalMem > 0) {
    qint64  usedMem    = totalMem - freeMem - buffers - cached;
    double  memPercent = 100.0 * usedMem / totalMem;
    QString memStr     = QString::number(memPercent, 'f', 1) + "%";
    setMemUsage(memStr);
  }
}

void DeviceStatus::readTimeUsed() {
  QFile file("/proc/uptime");
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    return;

  QTextStream in(&file);
  QString     line = in.readLine();
  file.close();

  QStringList parts = line.split(' ', Qt::SkipEmptyParts);
  if (parts.isEmpty())
    return;

  bool   ok;
  double uptimeSeconds = parts[0].toDouble(&ok);
  if (!ok)
    return;

  int totalSeconds = static_cast<int>(uptimeSeconds);
  int hours        = totalSeconds / 3600;
  int minutes      = (totalSeconds % 3600) / 60;
  int seconds      = totalSeconds % 60;

  QString timeStr = QString("%1:%2:%3")
                      .arg(hours, 2, 10, QChar('0'))
                      .arg(minutes, 2, 10, QChar('0'))
                      .arg(seconds, 2, 10, QChar('0'));

  setTimeUsed(timeStr);
}

void DeviceStatus::readSystemVolume() {
  // Try amixer first (ALSA)
  QProcess process;
  process.start("amixer",
                QStringList() << "-D"
                              << "pulse"
                              << "get"
                              << "Master");
  if (process.waitForFinished(1000)) {
    QByteArray output    = process.readAllStandardOutput();
    QString    outputStr = QString::fromUtf8(output);

    // Parse percentage from amixer output (e.g., "Front Left: Playback 50
    // [50%]")
    QRegExp rx("\\[([0-9]+)%\\]");
    if (rx.indexIn(outputStr) != -1) {
      bool   ok;
      double volume = rx.cap(1).toDouble(&ok);
      if (ok) {
        setVolumeInternal(volume);
        return;
      }
    }
  }

  // Fallback: try pactl (PulseAudio)
  process.start("pactl",
                QStringList() << "list"
                              << "sinks"
                              << "short");
  if (process.waitForFinished(1000)) {
    QByteArray output    = process.readAllStandardOutput();
    QString    outputStr = QString::fromUtf8(output);

    // Try to get default sink volume
    process.start("pactl",
                  QStringList() << "get-sink-volume"
                                << "@DEFAULT_SINK@");
    if (process.waitForFinished(1000)) {
      output    = process.readAllStandardOutput();
      outputStr = QString::fromUtf8(output);

      // Parse percentage (e.g., "Volume: front-left: 32768 /  50%")
      QRegExp rx("/([0-9]+)%");
      if (rx.indexIn(outputStr) != -1) {
        bool   ok;
        double volume = rx.cap(1).toDouble(&ok);
        if (ok) {
          setVolumeInternal(volume);
          return;
        }
      }
    }
  }

  // If both fail, keep default value
  qDebug() << "Could not read system volume, using default:" << m_volume;
}

void DeviceStatus::setCpuUsage(const QString& value) {
  if (m_cpuUsage == value)
    return;
  m_cpuUsage = value;
  emit cpuUsageChanged();
}

void DeviceStatus::setRamUsage(const QString& value) {
  if (m_ramUsage == value)
    return;
  m_ramUsage = value;
  emit ramUsageChanged();
}

void DeviceStatus::setTemperature(const QString& value) {
  if (m_temperature == value)
    return;
  m_temperature = value;
  emit temperatureChanged();
}

void DeviceStatus::setMemUsage(const QString& value) {
  if (m_memUsage == value)
    return;
  m_memUsage = value;
  emit memUsageChanged();
}

void DeviceStatus::setTimeUsed(const QString& value) {
  if (m_timeUsed == value)
    return;
  m_timeUsed = value;
  emit timeUsedChanged();
}

// Settings setters
void DeviceStatus::setVolumeInternal(double value) {
  if (qFuzzyCompare(m_volume, value))
    return;
  m_volume = value;
  emit volumeChanged();
}

void DeviceStatus::setVolume(double volume) {
  // Update internal value
  setVolumeInternal(volume);

  // Set system volume
  // Try amixer first (ALSA)
  QProcess process;
  int      volumePercent = static_cast<int>(volume);

  //   amixer -D pulse sset Master 50%
  process.start("amixer",
                QStringList()
                  << "-D"
                  << "pulse"
                  << "sset"
                  << "Master" << QString::number(volumePercent) + "%");
  if (process.waitForFinished(1000)) {
    return;
  }

  // Fallback: try pactl (PulseAudio)
  process.start("pactl",
                QStringList()
                  << "set-sink-volume"
                  << "@DEFAULT_SINK@" << QString::number(volumePercent) + "%");
  if (process.waitForFinished(1000)) {
    qDebug() << "Set volume using pactl:" << volumePercent << "%";
    return;
  }

  qDebug() << "Failed to set system volume";
}