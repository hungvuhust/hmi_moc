import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Rectangle {
    id: root
    Layout.fillWidth: true
    Layout.minimumHeight: 60
    Layout.maximumHeight: 60
    
    // Gradient background
    gradient: Gradient {
        GradientStop { position: 0.0; color: "#ffffff" }
        GradientStop { position: 1.0; color: "#f5f5f5" }
    }
    
    // Shadow effect
    Rectangle {
        anchors.fill: parent
        anchors.topMargin: parent.height
        height: 3
        gradient: Gradient {
            GradientStop { position: 0.0; color: "#40000000" }
            GradientStop { position: 1.0; color: "transparent" }
        }
    }

    property string operationMode: "AUTO"
    property real batteryLevel: 80

    RowLayout {
        anchors.fill: parent
        anchors.leftMargin: 30
        anchors.rightMargin: 30
        spacing: 20

        Image {
            id: logo
            Layout.minimumWidth: 100
            Layout.maximumWidth: 100
            Layout.minimumHeight: 45
            Layout.maximumHeight: 45
            source: "qrc:/newPrefix/rtc.png"
            fillMode: Image.PreserveAspectFit
        }

        Item {
            Layout.fillWidth: true
        }

        // Operation Mode Badge
        Rectangle {
            Layout.preferredWidth: 170
            Layout.preferredHeight: 40
            radius: 25
            color: {
                if (operationMode === "AUTO") return "#10b981"
                if (operationMode === "MANUAL") return "#f59e0b"
                return "#3b82f6"
            }
            
            RowLayout {
                anchors.centerIn: parent
                spacing: 10
                
                FontAwesome {
                    icon: operationMode === "AUTO" ? "circle-check" : (operationMode === "MANUAL" ? "circle-stop" : "gauge")
                    size: 16
                    color: "white"
                }
                
                Text {
                    text: operationMode
                    font.pixelSize: 16
                    font.bold: true
                    color: "white"
                }
            }
        }

        Item {
            Layout.fillWidth: true
        }

        // Date Time Card
        Rectangle {
            Layout.minimumWidth: 140
            Layout.preferredHeight: 40
            radius: 8
            color: "white"
            border.color: "#e5e7eb"
            border.width: 1
            
            ColumnLayout {
                anchors.centerIn: parent
                spacing: 2
                
                Text {
                    Layout.alignment: Qt.AlignHCenter
                    text: Qt.formatDateTime(new Date(), "dd/MM/yyyy")
                    font.pixelSize: 10
                    font.bold: true
                    color: "#7e270b"
                }
                
                Text {
                    id: timeText
                    Layout.alignment: Qt.AlignHCenter
                    text: Qt.formatDateTime(new Date(), "hh:mm:ss")
                    font.pixelSize: 13
                    font.bold: true
                    color: "#2e3436"
                }
            }

            Timer {
                interval: 1000
                running: true
                repeat: true
                onTriggered: {
                    timeText.text = Qt.formatDateTime(new Date(), "hh:mm:ss")
                }
            }
        }

        // Battery Indicator
        Rectangle {
            Layout.minimumWidth: 110
            Layout.preferredHeight: 40
            radius: 8
            color: "white"
            border.color: "#e5e7eb"
            border.width: 1
            
            RowLayout {
                anchors.fill: parent
                anchors.margins: 8
                spacing: 8
                
                FontAwesome {
                    Layout.alignment: Qt.AlignVCenter
                    icon: "battery-full"
                    size: 20
                    color: batteryLevel > 60 ? "#10b981" : (batteryLevel > 30 ? "#fbbf24" : "#ef4444")
                }
                
                ColumnLayout {
                    Layout.fillWidth: true
                    spacing: 1
                    
                    Text {
                        text: Math.round(batteryLevel) + "%"
                        font.pixelSize: 13
                        font.bold: true
                        color: "#2e3436"
                    }
                    
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.preferredHeight: 5
                        radius: 3
                        color: "#e5e7eb"
                        
                        Rectangle {
                            width: parent.width * (batteryLevel / 100.0)
                            height: parent.height
                            radius: 3
                            color: batteryLevel > 60 ? "#10b981" : (batteryLevel > 30 ? "#fbbf24" : "#ef4444")
                            
                            Behavior on width {
                                NumberAnimation { duration: 300 }
                            }
                        }
                    }
                }
            }
        }
    }
}

