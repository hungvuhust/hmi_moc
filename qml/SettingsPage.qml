import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    signal volumeChanged(real volume)
    signal maxVelocityChanged(real velocity)
    
    Component.onCompleted: {
        if (typeof deviceStatus !== "undefined" && deviceStatus) {
            volumeSlider.value = deviceStatus.volume
        }
        if (typeof robotData !== "undefined" && robotData) {
            velocitySlider.value = robotData.maxVelocity
        }
    }
    
    RowLayout {
        anchors.fill: parent
        anchors.margins: 20
        spacing: 20

        // Left Panel - Settings
        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: parent.width * 0.55

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
                color: "#ffffff"
                radius: 12
                border.color: "#e5e7eb"
                border.width: 1
                z: 1
            }

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 20
                spacing: 20
                z: 2

                // Title
                Text {
                    Layout.fillWidth: true
                    text: "Cài đặt hệ thống"
                    font.pixelSize: 24
                    font.bold: true
                    color: "#1f2937"
                }

                // Volume Control Section
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 100
                    color: "#f9fafb"
                    radius: 8
                    border.color: "#e5e7eb"
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 15
                        spacing: 10

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 12

                            Rectangle {
                                width: 40
                                height: 40
                                radius: 8
                                color: "#fef3c7"
                                
                                FontAwesome {
                                    anchors.centerIn: parent
                                    icon: "volume-high"
                                    size: 24
                                    color: "#d97706"
                                }
                            }

                            Text {
                                Layout.fillWidth: true
                                text: "Âm lượng"
                                font.pixelSize: 16
                                font.bold: true
                                color: "#374151"
                            }

                            Text {
                                text: Math.round(volumeSlider.value) + "%"
                                font.pixelSize: 16
                                font.bold: true
                                color: "#059669"
                                width: 50
                                horizontalAlignment: Text.AlignRight
                            }
                        }

                        Slider {
                            id: volumeSlider
                            Layout.fillWidth: true
                            from: 0
                            to: 100
                            value: 50
                            
                            onValueChanged: {
                                root.volumeChanged(value)
                            }
                            
                            Connections {
                                target: typeof deviceStatus !== "undefined" ? deviceStatus : null
                                function onVolumeChanged() {
                                    if (typeof deviceStatus !== "undefined" && deviceStatus && Math.abs(volumeSlider.value - deviceStatus.volume) > 0.1) {
                                        volumeSlider.value = deviceStatus.volume
                                    }
                                }
                            }
                            
                            background: Rectangle {
                                x: volumeSlider.leftPadding
                                y: volumeSlider.topPadding + volumeSlider.availableHeight / 2 - height / 2
                                implicitWidth: 200
                                implicitHeight: 6
                                width: volumeSlider.availableWidth
                                height: implicitHeight
                                radius: 3
                                color: "#e5e7eb"

                                Rectangle {
                                    width: volumeSlider.visualPosition * parent.width
                                    height: parent.height
                                    color: "#059669"
                                    radius: 3
                                }
                            }

                            handle: Rectangle {
                                x: volumeSlider.leftPadding + volumeSlider.visualPosition * (volumeSlider.availableWidth - width)
                                y: volumeSlider.topPadding + volumeSlider.availableHeight / 2 - height / 2
                                implicitWidth: 20
                                implicitHeight: 20
                                radius: 10
                                color: volumeSlider.pressed ? "#047857" : "#059669"
                                border.color: "#ffffff"
                                border.width: 2
                            }
                        }
                    }
                }

                // Velocity Settings Section
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 100
                    color: "#f9fafb"
                    radius: 8
                    border.color: "#e5e7eb"
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 15
                        spacing: 10

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 12

                            Rectangle {
                                width: 40
                                height: 40
                                radius: 8
                                color: "#dbeafe"
                                
                                FontAwesome {
                                    anchors.centerIn: parent
                                    icon: "gauge"
                                    size: 24
                                    color: "#2563eb"
                                }
                            }

                            Text {
                                Layout.fillWidth: true
                                text: "Tốc độ tối đa"
                                font.pixelSize: 16
                                font.bold: true
                                color: "#374151"
                            }

                            Text {
                                text: velocitySlider.value.toFixed(1) + " m/s"
                                font.pixelSize: 16
                                font.bold: true
                                color: "#2563eb"
                                width: 80
                                horizontalAlignment: Text.AlignRight
                            }
                        }

                        Slider {
                            id: velocitySlider
                            Layout.fillWidth: true
                            from: 0.4
                            to: 0.8
                            stepSize: 0.1
                            value: 0.5
                            
                            onValueChanged: {
                                root.maxVelocityChanged(value)
                            }
                            
                            Connections {
                                target: typeof robotData !== "undefined" ? robotData : null
                                function onMaxVelocityChanged() {
                                    if (typeof robotData !== "undefined" && robotData && Math.abs(velocitySlider.value - robotData.maxVelocity) > 0.01) {
                                        velocitySlider.value = robotData.maxVelocity
                                    }
                                }
                            }
                            
                            background: Rectangle {
                                x: velocitySlider.leftPadding
                                y: velocitySlider.topPadding + velocitySlider.availableHeight / 2 - height / 2
                                implicitWidth: 200
                                implicitHeight: 6
                                width: velocitySlider.availableWidth
                                height: implicitHeight
                                radius: 3
                                color: "#e5e7eb"

                                Rectangle {
                                    width: velocitySlider.visualPosition * parent.width
                                    height: parent.height
                                    color: "#2563eb"
                                    radius: 3
                                }
                            }

                            handle: Rectangle {
                                x: velocitySlider.leftPadding + velocitySlider.visualPosition * (velocitySlider.availableWidth - width)
                                y: velocitySlider.topPadding + velocitySlider.availableHeight / 2 - height / 2
                                implicitWidth: 20
                                implicitHeight: 20
                                radius: 10
                                color: velocitySlider.pressed ? "#1d4ed8" : "#2563eb"
                                border.color: "#ffffff"
                                border.width: 2
                            }
                        }
                    }
                }
            }
        }

        // Right Panel - IP Address Table
        Item {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: parent.width * 0.45

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
                color: "#ffffff"
                radius: 12
                border.color: "#e5e7eb"
                border.width: 1
                z: 1
            }

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 20
                spacing: 15
                z: 2

                // Title
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 12

                    Rectangle {
                        width: 40
                        height: 40
                        radius: 8
                        color: "#fce7f3"
                        
                        FontAwesome {
                            anchors.centerIn: parent
                            icon: "server"
                            size: 24
                            color: "#db2777"
                        }
                    }

                    Text {
                        Layout.fillWidth: true
                        text: "Thông tin mạng"
                        font.pixelSize: 24
                        font.bold: true
                        color: "#1f2937"
                    }
                }

                // Table Header
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    color: "#f3f4f6"
                    radius: 8

                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 12
                        spacing: 8

                        Text {
                            Layout.preferredWidth: 100
                            text: "Interface"
                            font.pixelSize: 13
                            font.bold: true
                            color: "#6b7280"
                        }

                        Text {
                            Layout.fillWidth: true
                            text: "IP Address"
                            font.pixelSize: 13
                            font.bold: true
                            color: "#6b7280"
                        }

                        Text {
                            Layout.preferredWidth: 80
                            text: "Status"
                            font.pixelSize: 13
                            font.bold: true
                            color: "#6b7280"
                            horizontalAlignment: Text.AlignRight
                        }
                    }
                }

                // Table Content
                ScrollView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    clip: true

                    ColumnLayout {
                        width: parent.width
                        spacing: 8

                        Repeater {
                            model: [
                                { interface: "eth0", ip: "192.168.1.100", status: "Connected" },
                                { interface: "wlan0", ip: "192.168.1.101", status: "Connected" },
                                { interface: "lo", ip: "127.0.0.1", status: "Active" },
                                { interface: "docker0", ip: "172.17.0.1", status: "Active" }
                            ]

                            Rectangle {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 50
                                color: index % 2 === 0 ? "#ffffff" : "#f9fafb"
                                radius: 6
                                border.color: "#e5e7eb"
                                border.width: 1

                                RowLayout {
                                    anchors.fill: parent
                                    anchors.margins: 12
                                    spacing: 8

                                    Rectangle {
                                        Layout.preferredWidth: 8
                                        Layout.preferredHeight: 8
                                        radius: 4
                                        color: modelData.status === "Connected" ? "#10b981" : "#6b7280"
                                    }

                                    Text {
                                        Layout.preferredWidth: 100
                                        text: modelData.interface
                                        font.pixelSize: 14
                                        font.bold: true
                                        color: "#1f2937"
                                    }

                                    Text {
                                        Layout.fillWidth: true
                                        text: modelData.ip
                                        font.pixelSize: 14
                                        color: "#374151"
                                    }

                                    Rectangle {
                                        Layout.preferredWidth: 80
                                        Layout.preferredHeight: 24
                                        radius: 12
                                        color: modelData.status === "Connected" ? "#d1fae5" : "#f3f4f6"
                                        
                                        Text {
                                            anchors.centerIn: parent
                                            text: modelData.status
                                            font.pixelSize: 11
                                            font.bold: true
                                            color: modelData.status === "Connected" ? "#059669" : "#6b7280"
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

