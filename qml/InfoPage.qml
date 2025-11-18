import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Dialogs 1.3

Item {
    id: root
    
    // Error Log Table
    Item {
        anchors.fill: parent
        anchors.margins: 10

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
                        color: "#fef3c7"
                        
                        FontAwesome {
                            anchors.centerIn: parent
                            icon: "triangle-exclamation"
                            size: 24
                            color: "#f59e0b"
                        }
                    }

                    Text {
                        Layout.fillWidth: true
                        text: "Error Log"
                        font.pixelSize: 24
                        font.bold: true
                        color: "#1f2937"
                    }
                    
                    Button {
                        Layout.preferredWidth: 120
                        Layout.preferredHeight: 36
                        text: "Clear Log"
                        font.pixelSize: 13
                        font.bold: true
                        
                        background: Rectangle {
                            radius: 8
                            color: parent.pressed ? "#dc2626" : (parent.hovered ? "#ef4444" : "#f87171")
                        }
                        
                        contentItem: RowLayout {
                            spacing: 6
                            anchors.centerIn: parent
                            
                            FontAwesome {
                                icon: "trash"
                                size: 14
                                color: "white"
                            }
                            
                            Text {
                                text: "Clear Log"
                                font.pixelSize: 13
                                font.bold: true
                                color: "white"
                            }
                        }
                        
                        onClicked: {
                            clearConfirmDialog.open()
                        }
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
                        anchors.margins: 10
                        spacing: 5

                        Text {
                            Layout.preferredWidth: 130
                            text: "Timestamp"
                            font.pixelSize: 12
                            font.bold: true
                            color: "#6b7280"
                        }

                        Text {
                            Layout.preferredWidth: 280
                            text: "Error Type"
                            font.pixelSize: 12
                            font.bold: true
                            color: "#6b7280"
                        }

                        Text {
                            Layout.preferredWidth: 250
                            text: "Description"
                            font.pixelSize: 12
                            font.bold: true
                            color: "#6b7280"
                        }

                        Text {
                            Layout.preferredWidth: 80
                            text: "Level"
                            font.pixelSize: 12
                            font.bold: true
                            color: "#6b7280"
                        }

                        Text {
                            Layout.preferredWidth: 80
                            text: "Duration"
                            font.pixelSize: 12
                            font.bold: true
                            color: "#6b7280"
                        }

                        Text {
                            Layout.preferredWidth: 90
                            text: "Status"
                            font.pixelSize: 12
                            font.bold: true
                            color: "#6b7280"
                        }
                        
                        Item {
                            Layout.fillWidth: true
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
                        spacing: 5

                        Repeater {
                            model: typeof robotData !== "undefined" && robotData ? robotData.errorLog : []

                            Rectangle {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 50
                                radius: 6
                                border.width: 2
                                
                                // Bôi đỏ nếu status = "active"
                                color: modelData.status === "active" ? "#fee2e2" : "#ffffff"
                                border.color: modelData.status === "active" ? "#ef4444" : "#e5e7eb"

                                RowLayout {
                                    anchors.fill: parent
                                    anchors.margins: 10
                                    spacing: 5

                                    Text {
                                        Layout.preferredWidth: 130
                                        text: modelData.timestamp
                                        font.pixelSize: 10
                                        color: modelData.status === "active" ? "#dc2626" : "#374151"
                                        font.bold: modelData.status === "active"
                                        elide: Text.ElideRight
                                        font.family: "monospace"
                                    }

                                    Text {
                                        Layout.preferredWidth: 280
                                        text: modelData.errorType
                                        font.pixelSize: 10
                                        color: modelData.status === "active" ? "#dc2626" : "#374151"
                                        font.bold: modelData.status === "active"
                                        elide: Text.ElideMiddle
                                    }

                                    // Scrolling Description
                                    Item {
                                        Layout.preferredWidth: 250
                                        Layout.fillHeight: true
                                        clip: true
                                        
                                        property bool isOverflow: descText.contentWidth > 250
                                        
                                        Text {
                                            id: descText
                                            y: parent.height / 2 - height / 2
                                            text: modelData.errorDescription
                                            font.pixelSize: 10
                                            color: modelData.status === "active" ? "#dc2626" : "#374151"
                                            font.bold: modelData.status === "active"
                                            wrapMode: Text.NoWrap
                                            
                                            SequentialAnimation {
                                                running: parent.isOverflow
                                                loops: Animation.Infinite
                                                
                                                PauseAnimation { duration: 2000 }
                                                NumberAnimation {
                                                    target: descText
                                                    property: "x"
                                                    from: 0
                                                    to: -(descText.contentWidth - 250 + 20)
                                                    duration: descText.contentWidth * 30
                                                    easing.type: Easing.Linear
                                                }
                                                PauseAnimation { duration: 1000 }
                                                NumberAnimation {
                                                    target: descText
                                                    property: "x"
                                                    to: 0
                                                    duration: 500
                                                    easing.type: Easing.InOutQuad
                                                }
                                            }
                                        }
                                    }

                                    Rectangle {
                                        Layout.preferredWidth: 80
                                        Layout.preferredHeight: 20
                                        radius: 10
                                        color: modelData.errorLevel === "FATAL" ? "#fee2e2" : "#fef3c7"
                                        border.color: modelData.errorLevel === "FATAL" ? "#ef4444" : "#f59e0b"
                                        border.width: 1
                                        
                                        Text {
                                            anchors.centerIn: parent
                                            text: modelData.errorLevel
                                            font.pixelSize: 9
                                            font.bold: true
                                            color: modelData.errorLevel === "FATAL" ? "#dc2626" : "#d97706"
                                        }
                                    }

                                    Text {
                                        Layout.preferredWidth: 80
                                        text: modelData.durationSeconds > 0 ? modelData.durationSeconds + "s" : "-"
                                        font.pixelSize: 10
                                        color: modelData.status === "active" ? "#dc2626" : "#374151"
                                        font.bold: modelData.status === "active"
                                        horizontalAlignment: Text.AlignRight
                                    }

                                    Rectangle {
                                        Layout.preferredWidth: 90
                                        Layout.preferredHeight: 24
                                        radius: 12
                                        color: modelData.status === "active" ? "#fee2e2" : "#d1fae5"
                                        border.color: modelData.status === "active" ? "#ef4444" : "#10b981"
                                        border.width: 1
                                        
                                        RowLayout {
                                            anchors.centerIn: parent
                                            spacing: 4
                                            
                                            Rectangle {
                                                width: 6
                                                height: 6
                                                radius: 3
                                                color: modelData.status === "active" ? "#ef4444" : "#10b981"
                                            }
                                            
                                            Text {
                                                text: modelData.status === "active" ? "Active" : "Cleared"
                                                font.pixelSize: 10
                                                font.bold: true
                                                color: modelData.status === "active" ? "#dc2626" : "#059669"
                                            }
                                        }
                                    }
                                    
                                    Item {
                                        Layout.fillWidth: true
                                    }
                                }
                            }
                        }
                    }
                }
                
                // Info text
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40
                    color: "#f9fafb"
                    radius: 8
                    border.color: "#e5e7eb"
                    border.width: 1
                    
                    RowLayout {
                        anchors.fill: parent
                        anchors.margins: 10
                        spacing: 8
                        
                        FontAwesome {
                            icon: "circle-info"
                            size: 16
                            color: "#6b7280"
                        }
                        
                        Text {
                            Layout.fillWidth: true
                            text: {
                                if (typeof robotData === "undefined" || !robotData) return "Lỗi đang hoạt động được highlight màu đỏ. Lỗi đã cleared hiển thị màu trắng."
                                
                                var allErrors = robotData.errorLog
                                var activeCount = 0
                                var clearedCount = 0
                                
                                for (var i = 0; i < allErrors.length; i++) {
                                    if (allErrors[i].status === "active") {
                                        activeCount++
                                    } else {
                                        clearedCount++
                                    }
                                }
                                
                                return "Tổng: " + allErrors.length + " lỗi | Active: " + activeCount + " | Cleared: " + clearedCount + " | Lỗi đang hoạt động được highlight màu đỏ."
                            }
                            font.pixelSize: 11
                            color: "#6b7280"
                            wrapMode: Text.WordWrap
                        }
                    }
                }
            }
        }
    
    // Clear Log Confirmation Dialog
    MessageDialog {
        id: clearConfirmDialog
        title: "Xác nhận xóa Error Log"
        text: "Bạn có chắc chắn muốn xóa toàn bộ error log?\n\nHành động này không thể hoàn tác."
        icon: StandardIcon.Warning
        standardButtons: StandardButton.Yes | StandardButton.No
        
        onYes: {
            if (typeof robotData !== "undefined" && robotData) {
                robotData.clearErrorLog()
            }
        }
    }
}
