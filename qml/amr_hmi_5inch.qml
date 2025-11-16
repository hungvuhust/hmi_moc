import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    id: mainWindow
    visible: true
    width: 1024
    height: 600
    minimumWidth: 1024
    minimumHeight: 600
    maximumWidth: 1024
    maximumHeight: 600
    title: "AMR Control System"
    color: "#1a1a1a"

    // Robot State Model
    QtObject {
        id: robotState
        property int battery: 55
        property string status: "idle" // idle, running, charging, error
        property real speed: 0.0
        property real posX: 12.5
        property real posY: 8.3
        property string task: "Standby"
        property int taskProgress: 0
        property bool obstacleDetected: false
        property int wifiStrength: 2
        property string mode: "auto" // auto, manual
    }

    // Timer for clock
    Timer {
        interval: 1000
        running: true
        repeat: true
        onTriggered: {
            currentTime.text = Qt.formatTime(new Date(), "hh:mm:ss")
        }
    }

    // Timer for robot simulation
    Timer {
        id: robotSimulation
        interval: 1000
        running: robotState.status === "running"
        repeat: true
        onTriggered: {
            robotState.speed = 0.8 + Math.random() * 0.4
            robotState.taskProgress = Math.min(robotState.taskProgress + 2, 100)
            robotState.posX += (Math.random() - 0.5) * 0.5
            robotState.posY += (Math.random() - 0.5) * 0.5
        }
    }

    // Main Container
    Rectangle {
        anchors.fill: parent
        gradient: Gradient {
            GradientStop { position: 0.0; color: "#1e293b" }
            GradientStop { position: 1.0; color: "#0f172a" }
        }

        // Header Status Bar
        Rectangle {
            id: header
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.right: parent.right
            height: 35
            color: "#0f172a"
            border.color: "#1e293b"
            border.width: 1

            RowLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 15

                // Status Indicator
                RowLayout {
                    spacing: 10
                    
                    Rectangle {
                        width: 10
                        height: 10
                        radius: 5
                        color: {
                            switch(robotState.status) {
                                case "running": return "#10b981"
                                case "error": return "#ef4444"
                                case "charging": return "#3b82f6"
                                default: return "#6b7280"
                            }
                        }

                        SequentialAnimation on opacity {
                            running: true
                            loops: Animation.Infinite
                            NumberAnimation { from: 1.0; to: 0.3; duration: 1000 }
                            NumberAnimation { from: 0.3; to: 1.0; duration: 1000 }
                        }
                    }

                    Text {
                        text: {
                            switch(robotState.status) {
                                case "running": return "Đang hoạt động"
                                case "error": return "Lỗi/Dừng khẩn"
                                case "charging": return "Đang sạc"
                                default: return "Chờ lệnh"
                            }
                        }
                        color: "white"
                        font.pixelSize: 12
                        font.bold: true
                    }
                }

                Item { Layout.fillWidth: true }

                // WiFi Indicator
                RowLayout {
                    spacing: 5
                    Rectangle {
                        width: 16
                        height: 12
                        color: "transparent"
                        Row {
                            anchors.bottom: parent.bottom
                            anchors.horizontalCenter: parent.horizontalCenter
                            spacing: 1
                            Repeater {
                                model: 4
                                Rectangle {
                                    width: 2
                                    height: 3 + index * 2
                                    color: index < robotState.wifiStrength ? "#10b981" : "#4b5563"
                                    radius: 1
                                    anchors.bottom: parent.bottom
                                }
                            }
                        }
                    }
                    Text {
                        text: robotState.wifiStrength + "/5"
                        color: "#9ca3af"
                        font.pixelSize: 11
                    }
                }

                // Battery Indicator
                RowLayout {
                    spacing: 5
                    Rectangle {
                        width: 20
                        height: 12
                        color: "transparent"
                        border.width: 1.5
                        border.color: {
                            if (robotState.battery > 60) return "#10b981"
                            if (robotState.battery > 30) return "#fbbf24"
                            return "#ef4444"
                        }
                        radius: 2
                        
                        Rectangle {
                            width: parent.width * (robotState.battery / 100)
                            height: parent.height - 2
                            anchors.left: parent.left
                            anchors.leftMargin: 1
                            anchors.verticalCenter: parent.verticalCenter
                            color: {
                                if (robotState.battery > 60) return "#10b981"
                                if (robotState.battery > 30) return "#fbbf24"
                                return "#ef4444"
                            }
                            radius: 1
                        }
                        
                        Rectangle {
                            width: 2
                            height: 6
                            anchors.left: parent.right
                            anchors.verticalCenter: parent.verticalCenter
                            color: parent.border.color
                            radius: 1
                        }
                    }
                    Text {
                        text: robotState.battery + "%"
                        color: {
                            if (robotState.battery > 60) return "#10b981"
                            if (robotState.battery > 30) return "#fbbf24"
                            return "#ef4444"
                        }
                        font.pixelSize: 11
                        font.bold: true
                    }
                }

                // Time
                Text {
                    id: currentTime
                    text: Qt.formatTime(new Date(), "hh:mm:ss")
                    color: "#9ca3af"
                    font.pixelSize: 11
                }
            }
        }

        // Main Content Area
        RowLayout {
            anchors.top: header.bottom
            anchors.bottom: footer.top
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.margins: 8
            spacing: 8

            // Left Panel - Map & Info
            ColumnLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                Layout.minimumWidth: 400
                spacing: 8

                // Mini Map
                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    color: "#0f172a"
                    radius: 6
                    border.color: "#1e293b"
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 12
                        spacing: 8

                        // Map Header
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 6
                            FontAwesome {
                                icon: "map"
                                size: 16
                                color: "white"
                            }
                            Text {
                                text: "Bản đồ"
                                color: "white"
                                font.pixelSize: 13
                                font.bold: true
                            }
                            Item { Layout.fillWidth: true }
                            Text {
                                text: "X: " + robotState.posX.toFixed(1) + "m | Y: " + robotState.posY.toFixed(1) + "m"
                                color: "#9ca3af"
                                font.pixelSize: 10
                            }
                        }

                        // Map Grid
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.fillHeight: true
                            color: "#0a0f1e"
                            radius: 4
                            border.color: "#1e293b"
                            border.width: 1

                            // Grid Lines
                            Grid {
                                anchors.fill: parent
                                columns: 8
                                rows: 8
                                Repeater {
                                    model: 64
                                    Rectangle {
                                        width: parent.width / 8
                                        height: parent.height / 8
                                        color: "transparent"
                                        border.color: "#374151"
                                        border.width: 0.5
                                        opacity: 0.3
                                    }
                                }
                            }

                            // Robot Position
                            Rectangle {
                                id: robotMarker
                                width: 16
                                height: 16
                                radius: 8
                                color: "#3b82f6"
                                border.color: "white"
                                border.width: 2
                                x: (robotState.posX / 20) * parent.width - width/2
                                y: (robotState.posY / 15) * parent.height - height/2

                                Behavior on x { NumberAnimation { duration: 500 } }
                                Behavior on y { NumberAnimation { duration: 500 } }

                                Rectangle {
                                    anchors.centerIn: parent
                                    width: parent.width
                                    height: parent.height
                                    radius: parent.radius
                                    color: "#60a5fa"
                                    opacity: 0.5

                                    SequentialAnimation on scale {
                                        running: true
                                        loops: Animation.Infinite
                                        NumberAnimation { from: 1.0; to: 1.8; duration: 1000 }
                                        NumberAnimation { from: 1.8; to: 1.0; duration: 1000 }
                                    }
                                }
                            }

                            // Destination Marker
                            Rectangle {
                                visible: robotState.status === "running"
                                width: 12
                                height: 12
                                radius: 2
                                color: "#10b981"
                                x: parent.width * 0.8 - width/2
                                y: parent.height * 0.3 - height/2

                                Rectangle {
                                    width: 8
                                    height: 8
                                    radius: 4
                                    color: "#34d399"
                                    anchors.centerIn: parent
                                    anchors.horizontalCenterOffset: 4
                                    anchors.verticalCenterOffset: -4

                                    SequentialAnimation on opacity {
                                        running: robotState.status === "running"
                                        loops: Animation.Infinite
                                        NumberAnimation { from: 1.0; to: 0.3; duration: 800 }
                                        NumberAnimation { from: 0.3; to: 1.0; duration: 800 }
                                    }
                                }
                            }
                        }
                    }
                }

                // Task Info
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 100
                    color: "#0f172a"
                    radius: 6
                    border.color: "#1e293b"
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 12
                        spacing: 8

                        Text {
                            text: "Nhiệm vụ hiện tại"
                            color: "white"
                            font.pixelSize: 13
                            font.bold: true
                        }

                        Text {
                            text: robotState.task
                            color: "#d1d5db"
                            font.pixelSize: 11
                        }

                        // Progress Bar
                        Rectangle {
                            Layout.fillWidth: true
                            height: 8
                            color: "#374151"
                            radius: 4

                            Rectangle {
                                width: parent.width * (robotState.taskProgress / 100)
                                height: parent.height
                                color: "#3b82f6"
                                radius: 4

                                Behavior on width { NumberAnimation { duration: 300 } }
                            }
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            Text {
                                text: "Tiến độ: " + robotState.taskProgress + "%"
                                color: "#9ca3af"
                                font.pixelSize: 10
                            }
                            Item { Layout.fillWidth: true }
                            Text {
                                text: "Tốc độ: " + robotState.speed.toFixed(1) + " m/s"
                                color: "#9ca3af"
                                font.pixelSize: 10
                            }
                        }
                    }
                }

                // Alert Box
                Rectangle {
                    visible: robotState.obstacleDetected
                    Layout.fillWidth: true
                    Layout.preferredHeight: 35
                    color: "#78350f"
                    border.color: "#fbbf24"
                    border.width: 1
                    radius: 6

                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 8
                        spacing: 8
                        FontAwesome {
                            icon: "triangle-exclamation"
                            size: 16
                            color: "#fef3c7"
                        }
                        Text {
                            text: "Phát hiện vật cản"
                            color: "#fef3c7"
                            font.pixelSize: 11
                        }
                    }
                }
            }

            // Right Panel - Controls
            ColumnLayout {
                Layout.preferredWidth: 180
                Layout.maximumWidth: 180
                Layout.minimumWidth: 170
                Layout.fillHeight: true
                spacing: 8

                // Mode Selection
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 70
                    color: "#0f172a"
                    radius: 6
                    border.color: "#1e293b"
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 8
                        spacing: 8

                        Text {
                            text: "Chế độ"
                            color: "white"
                            font.pixelSize: 11
                            font.bold: true
                        }

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 4

                            Button {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 30
                                text: "Tự động"
                                
                                background: Rectangle {
                                    color: robotState.mode === "auto" ? "#2563eb" : "#374151"
                                    radius: 4
                                }
                                
                                contentItem: Text {
                                    text: parent.text
                                    color: "white"
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    font.pixelSize: 10
                                }

                                onClicked: robotState.mode = "auto"
                            }

                            Button {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 30
                                text: "Thủ công"
                                
                                background: Rectangle {
                                    color: robotState.mode === "manual" ? "#2563eb" : "#374151"
                                    radius: 4
                                }
                                
                                contentItem: Text {
                                    text: parent.text
                                    color: "white"
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                    font.pixelSize: 10
                                }

                                onClicked: robotState.mode = "manual"
                            }
                        }
                    }
                }

                // Main Controls
                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    color: "#0f172a"
                    radius: 6
                    border.color: "#1e293b"
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 8
                        spacing: 8

                        Text {
                            text: "Điều khiển"
                            color: "white"
                            font.pixelSize: 11
                            font.bold: true
                        }

                        // Start Button
                        Button {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 40
                            enabled: robotState.status !== "running" && robotState.status !== "error"
                            
                            background: Rectangle {
                                color: parent.enabled ? (parent.pressed ? "#059669" : "#10b981") : "#4b5563"
                                radius: 6
                            }
                            
                            contentItem: RowLayout {
                                spacing: 8
                                Item { Layout.fillWidth: true }
                                Canvas {
                                    width: 14
                                    height: 14
                                    onPaint: {
                                        var ctx = getContext("2d");
                                        ctx.fillStyle = "white";
                                        ctx.beginPath();
                                        ctx.moveTo(0, 0);
                                        ctx.lineTo(14, 7);
                                        ctx.lineTo(0, 14);
                                        ctx.closePath();
                                        ctx.fill();
                                    }
                                }
                                Text {
                                    text: "Bắt đầu"
                                    color: "white"
                                    font.pixelSize: 13
                                    font.bold: true
                                }
                                Item { Layout.fillWidth: true }
                            }

                            onClicked: {
                                robotState.status = "running"
                                robotState.task = "Navigating to Point B"
                                robotState.taskProgress = 0
                            }
                        }

                        // Stop Button
                        Button {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 40
                            enabled: robotState.status === "running"
                            
                            background: Rectangle {
                                color: parent.enabled ? (parent.pressed ? "#1d4ed8" : "#2563eb") : "#4b5563"
                                radius: 6
                            }
                            
                            contentItem: RowLayout {
                                spacing: 8
                                Item { Layout.fillWidth: true }
                                Rectangle {
                                    width: 12
                                    height: 12
                                    color: "white"
                                    radius: 2
                                }
                                Text {
                                    text: "Dừng"
                                    color: "white"
                                    font.pixelSize: 13
                                    font.bold: true
                                }
                                Item { Layout.fillWidth: true }
                            }

                            onClicked: {
                                robotState.status = "idle"
                                robotState.speed = 0
                                robotState.task = "Standby"
                            }
                        }

                        // Emergency Stop Button
                        Button {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 40
                            
                            background: Rectangle {
                                color: parent.pressed ? "#b91c1c" : "#dc2626"
                                radius: 6
                            }
                            
                            contentItem: RowLayout {
                                spacing: 8
                                Item { Layout.fillWidth: true }
                                FontAwesome {
                                    icon: "bell"
                                    size: 16
                                    color: "white"
                                }
                                Text {
                                    text: "DỪNG KHẨN"
                                    color: "white"
                                    font.pixelSize: 13
                                    font.bold: true
                                }
                                Item { Layout.fillWidth: true }
                            }

                            onClicked: {
                                robotState.status = "error"
                                robotState.speed = 0
                                robotState.task = "Emergency Stop Activated"
                            }
                        }

                        // Reset Button
                        Button {
                            visible: robotState.status === "error"
                            Layout.fillWidth: true
                            Layout.preferredHeight: 35
                            
                            background: Rectangle {
                                color: parent.pressed ? "#b45309" : "#d97706"
                                radius: 6
                            }
                            
                            contentItem: RowLayout {
                                spacing: 6
                                Item { Layout.fillWidth: true }
                                Canvas {
                                    width: 14
                                    height: 14
                                    onPaint: {
                                        var ctx = getContext("2d");
                                        ctx.strokeStyle = "white";
                                        ctx.lineWidth = 2;
                                        ctx.beginPath();
                                        ctx.arc(7, 7, 5, 0.5 * Math.PI, 2.3 * Math.PI);
                                        ctx.stroke();
                                        // Arrow head
                                        ctx.beginPath();
                                        ctx.moveTo(12, 7);
                                        ctx.lineTo(10, 4);
                                        ctx.lineTo(8, 7);
                                        ctx.fill();
                                    }
                                }
                                Text {
                                    text: "Khởi động lại"
                                    color: "white"
                                    font.pixelSize: 10
                                    font.bold: true
                                }
                                Item { Layout.fillWidth: true }
                            }

                            onClicked: {
                                robotState.status = "idle"
                                robotState.speed = 0
                                robotState.task = "Standby"
                                robotState.taskProgress = 0
                            }
                        }
                    }
                }

                // Quick Actions
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 75
                    color: "#0f172a"
                    radius: 6
                    border.color: "#1e293b"
                    border.width: 1

                    GridLayout {
                        anchors.fill: parent
                        anchors.margins: 8
                        columns: 2
                        rowSpacing: 4
                        columnSpacing: 4

                        Button {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 28
                            
                            background: Rectangle {
                                color: parent.pressed ? "#4b5563" : "#374151"
                                radius: 4
                            }
                            
                            contentItem: RowLayout {
                                spacing: 4
                                Item { Layout.fillWidth: true }
                                FontAwesome {
                                    icon: "home"
                                    size: 16
                                    color: "white"
                                }
                                Text {
                                    text: "Home"
                                    color: "white"
                                    font.pixelSize: 9
                                }
                                Item { Layout.fillWidth: true }
                            }
                        }

                        Button {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 28
                            
                            background: Rectangle {
                                color: parent.pressed ? "#4b5563" : "#374151"
                                radius: 4
                            }
                            
                            contentItem: RowLayout {
                                spacing: 4
                                Item { Layout.fillWidth: true }
                                FontAwesome {
                                    icon: "location"
                                    size: 16
                                    color: "white"
                                }
                                Text {
                                    text: "Định vị"
                                    color: "white"
                                    font.pixelSize: 9
                                }
                                Item { Layout.fillWidth: true }
                            }
                        }

                        Button {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 28
                            Layout.columnSpan: 2
                            
                            background: Rectangle {
                                color: parent.pressed ? "#4b5563" : "#374151"
                                radius: 4
                            }
                            
                            contentItem: RowLayout {
                                spacing: 4
                                Item { Layout.fillWidth: true }
                                FontAwesome {
                                    icon: "gear"
                                    size: 16
                                    color: "white"
                                }
                                Text {
                                    text: "Cài đặt"
                                    color: "white"
                                    font.pixelSize: 9
                                }
                                Item { Layout.fillWidth: true }
                            }
                        }
                    }
                }
            }
        }

        // Footer
        Rectangle {
            id: footer
            anchors.bottom: parent.bottom
            anchors.left: parent.left
            anchors.right: parent.right
            height: 30
            color: "#0f172a"
            border.color: "#1e293b"
            border.width: 1

            RowLayout {
                anchors.fill: parent
                anchors.margins: 8

                Text {
                    text: "AMR-001 v2.0"
                    color: "#9ca3af"
                    font.pixelSize: 10
                }

                Item { Layout.fillWidth: true }

                Text {
                    text: "© 2025 Robot Control System"
                    color: "#9ca3af"
                    font.pixelSize: 10
                }
            }
        }
    }
}
