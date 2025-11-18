import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    
    signal startClicked()
    signal stopClicked()
    signal relocationClicked()
    
    property real batteryLevel: 0.2
    property string robotState: "IDLE"
    property real posX: 0.0
    property real posY: 0.0
    property real theta: 0.0
    property real speed: 0.0
    property string mapName: "Map current"
    property string destination: "None"
    property real localizationScore: 0.0
    property string cpuUsage: "0%"
    property string ramUsage: "0%"
    property string temperature: "0°C"
    property string memUsage: "0%"
    property string timeUsed: "00:00:00"
    
    // Device error status
    property bool emgFrontError: false
    property bool emgRearError: false
    property bool motorLeftError: false
    property bool motorRightError: false
    property bool lidarFrontError: false
    property bool lidarRearError: false
    property bool bumperError: false

    RowLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        // Data Frame
        Item {
            Layout.minimumWidth: 250
            Layout.maximumWidth: 250
            Layout.fillHeight: true

            // Shadow
            Rectangle {
                anchors.fill: parent
                anchors.topMargin: 4
                anchors.leftMargin: 2
                color: "#20000000"
                radius: 12
                z: 0
            }

            Rectangle {
                anchors.fill: parent
                color: "#f9fafb"
                radius: 12
                border.color: "#e5e7eb"
                border.width: 1
                z: 1
            }

            ColumnLayout {
                anchors.centerIn: parent
                width: parent.width - 20
                spacing: 8
                z: 2

                // Position (X, Y, Theta)
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 32
                    color: "#ffffff"
                    border.color: "#e5e7eb"
                    border.width: 1
                    radius: 8

                    RowLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 8
                        anchors.rightMargin: 8
                        spacing: 8

                        FontAwesome {
                            icon: "location"
                            size: 14
                            color: "#7e270b"
                        }

                        Text {
                            Layout.fillWidth: true
                            text: "X: " + posX.toFixed(2) + " | Y: " + posY.toFixed(2) + " | θ: " + theta.toFixed(1) + "°"
                            font.pixelSize: 13
                            font.bold: true
                            verticalAlignment: Text.AlignVCenter
                            wrapMode: Text.NoWrap
                        }
                    }
                }

                // Speed
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 32
                    color: "#ffffff"
                    border.color: "#e5e7eb"
                    border.width: 1
                    radius: 8

                    RowLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 8
                        anchors.rightMargin: 8
                        spacing: 8

                        FontAwesome {
                            icon: "bolt"
                            size: 14
                            color: "#7e270b"
                        }

                        Text {
                            Layout.fillWidth: true
                            text: "Tốc độ: " + speed.toFixed(2) + " m/s"
                            font.pixelSize: 13
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                }

                // Map
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 32
                    color: "#ffffff"
                    border.color: "#e5e7eb"
                    border.width: 1
                    radius: 8

                    RowLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 8
                        anchors.rightMargin: 8
                        spacing: 8

                        FontAwesome {
                            icon: "map"
                            size: 14
                            color: "#7e270b"
                        }

                        Text {
                            Layout.fillWidth: true
                            text: "Map: " + mapName
                            font.pixelSize: 13
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                }

                // Destination
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 32
                    color: "#ffffff"
                    border.color: "#e5e7eb"
                    border.width: 1
                    radius: 8

                    RowLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 8
                        anchors.rightMargin: 8
                        spacing: 8

                        FontAwesome {
                            icon: "bullseye"
                            size: 14
                            color: "#7e270b"
                        }

                        Text {
                            Layout.fillWidth: true
                            text: "Đích: " + destination
                            font.pixelSize: 13
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                }

                // Localization Score
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 32
                    color: localizationScore > 0.8 ? "#ecfdf5" : (localizationScore > 0.5 ? "#fef3c7" : "#fee2e2")
                    border.color: localizationScore > 0.8 ? "#10b981" : (localizationScore > 0.5 ? "#fbbf24" : "#ef4444")
                    border.width: 1
                    radius: 8

                    RowLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 8
                        anchors.rightMargin: 8
                        spacing: 8

                        FontAwesome {
                            icon: "signal"
                            size: 14
                            color: localizationScore > 0.8 ? "#10b981" : (localizationScore > 0.5 ? "#fbbf24" : "#ef4444")
                        }

                        Text {
                            Layout.fillWidth: true
                            text: "Định vị: " + localizationScore.toFixed(2) + "%"
                            font.pixelSize: 13
                            font.bold: true
                            verticalAlignment: Text.AlignVCenter
                            color: localizationScore > 0.8 ? "#10b981" : (localizationScore > 0.5 ? "#fbbf24" : "#ef4444")
                        }
                    }
                }

                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 2
                    color: "#7e270b"
                }

                // CPU & RAM
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 32
                    color: "#ffffff"
                    border.color: "#e5e7eb"
                    border.width: 1
                    radius: 8

                    RowLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 8
                        anchors.rightMargin: 8
                        spacing: 6

                        FontAwesome {
                            icon: "microchip"
                            size: 14
                            color: "#7e270b"
                        }

                        Text {
                            text: "CPU: " + cpuUsage
                            font.pixelSize: 13
                            verticalAlignment: Text.AlignVCenter
                        }

                        Text {
                            text: "|"
                            font.pixelSize: 13
                            verticalAlignment: Text.AlignVCenter
                        }

                        FontAwesome {
                            icon: "memory"
                            size: 14
                            color: "#7e270b"
                        }

                        Text {
                            text: "RAM: " + ramUsage
                            font.pixelSize: 13
                            verticalAlignment: Text.AlignVCenter
                        }

                        Item {
                            Layout.fillWidth: true
                        }
                    }
                }

                // Temperature & Memory
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 32
                    color: "#ffffff"
                    border.color: "#e5e7eb"
                    border.width: 1
                    radius: 8

                    RowLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 8
                        anchors.rightMargin: 8
                        spacing: 6

                        FontAwesome {
                            icon: "thermometer"
                            size: 14
                            color: "#7e270b"
                        }

                        Text {
                            text: "Temp: " + temperature
                            font.pixelSize: 13
                            verticalAlignment: Text.AlignVCenter
                        }

                        Text {
                            text: "|"
                            font.pixelSize: 13
                            verticalAlignment: Text.AlignVCenter
                        }

                        FontAwesome {
                            icon: "hard-drive"
                            size: 14
                            color: "#7e270b"
                        }

                        Text {
                            text: "ROM: " + memUsage
                            font.pixelSize: 13
                            verticalAlignment: Text.AlignVCenter
                        }

                        Item {
                            Layout.fillWidth: true
                        }
                    }
                }

                // Time Used
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 32
                    color: "#ffffff"
                    border.color: "#e5e7eb"
                    border.width: 1
                    radius: 8

                    RowLayout {
                        anchors.fill: parent
                        anchors.leftMargin: 8
                        anchors.rightMargin: 8
                        spacing: 8

                        FontAwesome {
                            icon: "clock"
                            size: 14
                            color: "#7e270b"
                        }

                        Text {
                            Layout.fillWidth: true
                            text: "Time Used: " + timeUsed
                            font.pixelSize: 13
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                }
            }
        }

        // Center Frame
        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true

            // Shadow
            Rectangle {
                anchors.fill: parent
                anchors.topMargin: 4
                anchors.leftMargin: 2
                color: "#20000000"
                radius: 12
                z: 0
            }

            Rectangle {
                anchors.fill: parent
                color: "#f9fafb"
                radius: 12
                border.color: "#e5e7eb"
                border.width: 1
                z: 1
            }

            ColumnLayout {
                anchors.centerIn: parent
                spacing: 20
                z: 2

                // Status Box
                Rectangle {
                    Layout.alignment: Qt.AlignHCenter
                    Layout.preferredWidth: 350
                    Layout.preferredHeight: 80
                    radius: 12
                    color: {
                        if (robotState === "IDLE") return "#d1f4e0"
                        if (robotState === "RUNNING" || robotState === "BUSY") return "#fef3c7"
                        if (robotState === "ERROR") return "#fee2e2"
                        return "#ffffff"
                    }
                    border.color: {
                        if (robotState === "IDLE") return "#10b981"
                        if (robotState === "RUNNING" || robotState === "BUSY") return "#f59e0b"
                        if (robotState === "ERROR") return "#ef4444"
                        return "#e5e7eb"
                    }
                    border.width: 2

                    Text {
                        anchors.centerIn: parent
                        text: robotState
                        font.pixelSize: 42
                        font.bold: true
                        color: {
                            if (robotState === "IDLE") return "#059669"
                            if (robotState === "RUNNING" || robotState === "BUSY") return "#d97706"
                            if (robotState === "ERROR") return "#dc2626"
                            return "#2e3436"
                        }
                    }
                }

                // AGV Canvas (theo URDF: 0.8m x 0.6m, scale 1m = 200px)
                Canvas {
                    id: agvCanvas
                    Layout.alignment: Qt.AlignHCenter
                    Layout.preferredWidth: 400
                    Layout.preferredHeight: 300

                    // Trigger repaint when error properties change
                    property bool _emgFrontError: root.emgFrontError
                    property bool _emgRearError: root.emgRearError
                    property bool _motorLeftError: root.motorLeftError
                    property bool _motorRightError: root.motorRightError
                    property bool _lidarFrontError: root.lidarFrontError
                    property bool _lidarRearError: root.lidarRearError
                    property bool _bumperError: root.bumperError

                    on_EmgFrontErrorChanged: requestPaint()
                    on_EmgRearErrorChanged: requestPaint()
                    on_MotorLeftErrorChanged: requestPaint()
                    on_MotorRightErrorChanged: requestPaint()
                    on_LidarFrontErrorChanged: requestPaint()
                    on_LidarRearErrorChanged: requestPaint()
                    on_BumperErrorChanged: requestPaint()

                    onPaint: {
                        var ctx = getContext("2d")
                        ctx.clearRect(0, 0, width, height)
                        
                        // Scale: 1m = 250px (to hơn)
                        var scale = 250
                        var centerX = width / 2  // Tâm xe
                        var centerY = height / 2
                        
                        // Kích thước xe: 0.8m x 0.6m (xoay dọc: chiều cao x chiều rộng)
                        var bodyHeight = 0.8 * scale  // 200px (dọc)
                        var bodyWidth = 0.6 * scale   // 150px (ngang)
                        var radius = 15
                        
                        var bodyX = centerX - bodyWidth / 2
                        var bodyY = centerY - bodyHeight / 2

                        // AGV Body (rounded rectangle - dọc)
                        ctx.fillStyle = '#ffffff'
                        ctx.strokeStyle = "#2e3436"
                        ctx.lineWidth = 3
                        
                        ctx.beginPath()
                        ctx.moveTo(bodyX + radius, bodyY)
                        ctx.lineTo(bodyX + bodyWidth - radius, bodyY)
                        ctx.quadraticCurveTo(bodyX + bodyWidth, bodyY, bodyX + bodyWidth, bodyY + radius)
                        ctx.lineTo(bodyX + bodyWidth, bodyY + bodyHeight - radius)
                        ctx.quadraticCurveTo(bodyX + bodyWidth, bodyY + bodyHeight, bodyX + bodyWidth - radius, bodyY + bodyHeight)
                        ctx.lineTo(bodyX + radius, bodyY + bodyHeight)
                        ctx.quadraticCurveTo(bodyX, bodyY + bodyHeight, bodyX, bodyY + bodyHeight - radius)
                        ctx.lineTo(bodyX, bodyY + radius)
                        ctx.quadraticCurveTo(bodyX, bodyY, bodyX + radius, bodyY)
                        ctx.closePath()
                        ctx.fill()
                        ctx.stroke()

                        // Bumper Frame (bao quanh robot body)
                        var bumperPadding = 12
                        var bumperRadius = radius + 8
                        var bumperX = bodyX - bumperPadding
                        var bumperY = bodyY - bumperPadding
                        var bumperWidth = bodyWidth + bumperPadding * 2
                        var bumperHeight = bodyHeight + bumperPadding * 2
                        
                        ctx.strokeStyle = bumperError ? "#ef4444" : "#4fce64"
                        ctx.lineWidth = bumperError ? 5 : 4
                        ctx.setLineDash([])
                        
                        ctx.beginPath()
                        ctx.moveTo(bumperX + bumperRadius, bumperY)
                        ctx.lineTo(bumperX + bumperWidth - bumperRadius, bumperY)
                        ctx.quadraticCurveTo(bumperX + bumperWidth, bumperY, bumperX + bumperWidth, bumperY + bumperRadius)
                        ctx.lineTo(bumperX + bumperWidth, bumperY + bumperHeight - bumperRadius)
                        ctx.quadraticCurveTo(bumperX + bumperWidth, bumperY + bumperHeight, bumperX + bumperWidth - bumperRadius, bumperY + bumperHeight)
                        ctx.lineTo(bumperX + bumperRadius, bumperY + bumperHeight)
                        ctx.quadraticCurveTo(bumperX, bumperY + bumperHeight, bumperX, bumperY + bumperHeight - bumperRadius)
                        ctx.lineTo(bumperX, bumperY + bumperRadius)
                        ctx.quadraticCurveTo(bumperX, bumperY, bumperX + bumperRadius, bumperY)
                        ctx.closePath()
                        ctx.stroke()
                        


                        // EMG Button 1 (front center - red circle)
                        var emgRadius = 18
                        ctx.fillStyle = emgFrontError ?"#ef4444" : '#4fce64'
                        ctx.strokeStyle =  "#2e3436"
                        ctx.lineWidth = emgFrontError ? 3 : 2
                        ctx.beginPath()
                        ctx.arc(centerX, bodyY + 30, emgRadius, 0, Math.PI * 2)
                        ctx.fill()
                        ctx.stroke()
                        
                        ctx.fillStyle = "black"
                        ctx.font = "bold 10px sans-serif"
                        ctx.textAlign = "center"
                        ctx.fillText("EMG", centerX, bodyY + 30 + 4)

                        // EMG Button 2 (rear center - red circle)
                        ctx.fillStyle = emgRearError ? "#ef4444" : '#4fce64'
                        ctx.strokeStyle =  "#2e3436"
                        ctx.lineWidth = emgRearError ? 3 : 2
                        ctx.beginPath()
                        ctx.arc(centerX, bodyY + bodyHeight - 30, emgRadius, 0, Math.PI * 2)
                        ctx.fill()
                        ctx.stroke()
                        
                        ctx.fillStyle = "black"
                        ctx.font = "bold 10px sans-serif"
                        ctx.textAlign = "center"
                        ctx.fillText("EMG", centerX, bodyY + bodyHeight - 30 + 4)

                        // Motor Left (cách tâm 0.21m)
                        var motorOffset = 0.21 * scale
                        var motorWidth = 30
                        var motorHeight = 40
                        
                        ctx.fillStyle = motorLeftError ? "#ef4444" : '#4fce64'
                        ctx.strokeStyle = motorLeftError ? "#dc2626" : "#000000"
                        ctx.lineWidth = motorLeftError ? 3 : 2
                        ctx.fillRect(centerX - motorOffset - motorWidth/2, centerY - motorHeight/2, motorWidth, motorHeight)
                        ctx.strokeRect(centerX - motorOffset - motorWidth/2, centerY - motorHeight/2, motorWidth, motorHeight)
                        
                        ctx.fillStyle = motorLeftError ? "#ffffff" : '#070707'
                        ctx.font = "bold 16px sans-serif"
                        ctx.textAlign = "center"
                        ctx.fillText("M", centerX - motorOffset, centerY + 5)

                        // Motor Right (cách tâm 0.21m)
                        ctx.fillStyle = motorRightError ? "#ef4444" : '#4fce64'
                        ctx.strokeStyle = motorRightError ? "#dc2626" : "#000000"
                        ctx.lineWidth = motorRightError ? 3 : 2
                        ctx.fillRect(centerX + motorOffset - motorWidth/2, centerY - motorHeight/2, motorWidth, motorHeight)
                        ctx.strokeRect(centerX + motorOffset - motorWidth/2, centerY - motorHeight/2, motorWidth, motorHeight)
                        
                        ctx.fillStyle = motorRightError ? "#ffffff" : '#070707'
                        ctx.font = "bold 16px sans-serif"
                        ctx.textAlign = "center"
                        ctx.fillText("M", centerX + motorOffset, centerY + 5)

                        // Front Lidar - góc phải trên
                        var frontLidarX = centerX + 0.20 * scale
                        var frontLidarY = centerY - 0.33 * scale
                        var lidarRadius = 30  // Bé hơn nữa
                        
                        // Quét từ phải sang trái-trên (270°)
                        var frontCenterAngle = -Math.PI / 4  // -45° (lên trên bên phải)
                        var frontStartAngle = frontCenterAngle - Math.PI * 3/4  // -180°
                        var frontEndAngle = frontCenterAngle + Math.PI * 3/4    // 90°
                        
                        ctx.fillStyle = lidarFrontError ? "rgba(239, 68, 68, 0.3)" : "rgba(59, 130, 246, 0.3)"
                        ctx.strokeStyle = lidarFrontError ? "#ef4444" : '#4fce64'
                        ctx.lineWidth = lidarFrontError ? 3 : 2
                        ctx.beginPath()
                        ctx.moveTo(frontLidarX, frontLidarY)
                        ctx.arc(frontLidarX, frontLidarY, lidarRadius, frontStartAngle, frontEndAngle)
                        ctx.lineTo(frontLidarX, frontLidarY)
                        ctx.closePath()
                        ctx.fill()
                        ctx.stroke()
                        
                        // Lidar center dot
                        ctx.fillStyle = lidarFrontError ?"#ef4444" : '#4fce64'
                        ctx.beginPath()
                        ctx.arc(frontLidarX, frontLidarY, 5, 0, Math.PI * 2)
                        ctx.fill()

                        // Rear Lidar - góc trái dưới
                        var rearLidarX = centerX - 0.20 * scale
                        var rearLidarY = centerY + 0.33 * scale
                        
                        // Quét từ trái sang phải-dưới (270°)
                        var rearCenterAngle = Math.PI * 3/4  // 135° (xuống dưới bên trái)
                        var rearStartAngle = rearCenterAngle - Math.PI * 3/4  // 0°
                        var rearEndAngle = rearCenterAngle + Math.PI * 3/4    // 270°
                        
                        ctx.fillStyle = lidarRearError ? "rgba(239, 68, 68, 0.3)" : "rgba(59, 130, 246, 0.3)"
                        ctx.strokeStyle = lidarRearError ? "#ef4444" : '#4fce64'
                        ctx.lineWidth = lidarRearError ? 3 : 2
                        ctx.beginPath()
                        ctx.moveTo(rearLidarX, rearLidarY)
                        ctx.arc(rearLidarX, rearLidarY, lidarRadius, rearStartAngle, rearEndAngle)
                        ctx.lineTo(rearLidarX, rearLidarY)
                        ctx.closePath()
                        ctx.fill()
                        ctx.stroke()
                        
                        // Lidar center dot
                        ctx.fillStyle = lidarRearError ?"#ef4444" : '#4fce64'
                        ctx.beginPath()
                        ctx.arc(rearLidarX, rearLidarY, 5, 0, Math.PI * 2)
                        ctx.fill()

                        // Direction arrow (phía trước - lên trên)
                        ctx.strokeStyle = "#ffffff"
                        ctx.fillStyle = "#ffffff"
                        ctx.lineWidth = 2
                        ctx.beginPath()
                        ctx.moveTo(centerX, bodyY - 10)
                        ctx.lineTo(centerX, bodyY - 25)
                        ctx.stroke()
                        
                        ctx.beginPath()
                        ctx.moveTo(centerX, bodyY - 25)
                        ctx.lineTo(centerX - 5, bodyY - 18)
                        ctx.lineTo(centerX + 5, bodyY - 18)
                        ctx.closePath()
                        ctx.fill()

                    }

                    Component.onCompleted: {
                        requestPaint()
                    }
                }
            }
        }

        // Control Frame
        Item {
            Layout.minimumWidth: 250
            Layout.maximumWidth: 250
            Layout.fillHeight: true

            // Shadow
            Rectangle {
                anchors.fill: parent
                anchors.topMargin: 4
                anchors.leftMargin: 2
                color: "#20000000"
                radius: 12
                z: 0
            }

            Rectangle {
                anchors.fill: parent
                color: "#f9fafb"
                radius: 12
                border.color: "#e5e7eb"
                border.width: 1
                z: 1
            }

            ColumnLayout {
                anchors.centerIn: parent
                spacing: 15
                z: 2

                // START Button
                Button {
                    id: startButton
                    Layout.alignment: Qt.AlignHCenter
                    Layout.minimumWidth: 220
                    Layout.maximumWidth: 220
                    Layout.minimumHeight: 80
                    Layout.maximumHeight: 80

                    background: Rectangle {
                        radius: 12
                        color: startButton.pressed ? '#10b981' : '#059669'
                        border.color: "#047857"
                        border.width: 2

                        Rectangle {
                            anchors.fill: parent
                            anchors.margins: 2
                            radius: 10
                            color: "transparent"
                            border.color: "#34d399"
                            border.width: 1
                            opacity: 0.3
                        }
                    }

                    contentItem: RowLayout {
                        spacing: 12

                        FontAwesome {
                            Layout.alignment: Qt.AlignVCenter
                            icon: "play"
                            size: 28
                            color: "white"
                        }

                        Text {
                            Layout.fillWidth: true
                            text: "START"
                            font.pixelSize: 28
                            font.bold: true
                            color: "white"
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                    
                    onClicked: {
                        root.startClicked()
                    }
                }

                // STOP Button
                Button {
                    id: stopButton
                    Layout.alignment: Qt.AlignHCenter
                    Layout.minimumWidth: 220
                    Layout.maximumWidth: 220
                    Layout.minimumHeight: 80
                    Layout.maximumHeight: 80

                    background: Rectangle {
                        radius: 12
                        color: stopButton.pressed ? '#f87171' : '#ef4444'
                        border.color: "#dc2626"
                        border.width: 2

                        Rectangle {
                            anchors.fill: parent
                            anchors.margins: 2
                            radius: 10
                            color: "transparent"
                            border.color: "#fca5a5"
                            border.width: 1
                            opacity: 0.3
                        }
                    }

                    contentItem: RowLayout {
                        spacing: 12

                        FontAwesome {
                            Layout.alignment: Qt.AlignVCenter
                            icon: "circle-stop"
                            size: 28
                            color: "white"
                        }

                        Text {
                            Layout.fillWidth: true
                            text: "STOP"
                            font.pixelSize: 28
                            font.bold: true
                            color: "white"
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                    
                    onClicked: {
                        root.stopClicked()
                    }
                }

                // RELOCATION Button
                Button {
                    id: relocationButton
                    Layout.alignment: Qt.AlignHCenter
                    Layout.minimumWidth: 220
                    Layout.maximumWidth: 220
                    Layout.minimumHeight: 80
                    Layout.maximumHeight: 80

                    background: Rectangle {
                        radius: 12
                        color: relocationButton.pressed ? '#fbbf24' : '#f59e0b'
                        border.color: "#d97706"
                        border.width: 2

                        Rectangle {
                            anchors.fill: parent
                            anchors.margins: 2
                            radius: 10
                            color: "transparent"
                            border.color: "#fcd34d"
                            border.width: 1
                            opacity: 0.3
                        }
                    }

                    contentItem: RowLayout {
                        spacing: 12

                        FontAwesome {
                            Layout.alignment: Qt.AlignVCenter
                            icon: "crosshairs"
                            size: 28
                            color: "white"
                        }

                        Text {
                            Layout.fillWidth: true
                            text: "RELOCATION"
                            font.pixelSize: 22
                            font.bold: true
                            color: "white"
                            horizontalAlignment: Text.AlignLeft
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                    
                    onClicked: {
                        root.relocationClicked()
                    }
                }
            }
        }
    }
}

