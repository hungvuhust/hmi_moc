import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    
    signal mapSelected(string mapName)
    signal addTagClicked()
    
    // Properties để nhận dữ liệu từ C++ backend
    property var mapList: []
    property string currentMapName: ""
    property real originX: 0.0
    property real originY: 0.0
    property real originTheta: 0.0

    RowLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        // Left Panel - Map List
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
                        color: "#dbeafe"
                        
                        FontAwesome {
                            anchors.centerIn: parent
                            icon: "map"
                            size: 24
                            color: "#2563eb"
                        }
                    }

                    Text {
                        Layout.fillWidth: true
                        text: "Danh sách bản đồ"
                        font.pixelSize: 24
                        font.bold: true
                        color: "#1f2937"
                    }
                }

                // Map List
                ScrollView {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    clip: true

                    ColumnLayout {
                        width: parent.width
                        spacing: 12

                        Repeater {
                            model: mapList

                            Rectangle {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 60
                                color: currentMapName === modelData ? "#eff6ff" : "#ffffff"
                                radius: 8
                                border.color: currentMapName === modelData ? "#3b82f6" : "#e5e7eb"
                                border.width: currentMapName === modelData ? 2 : 1

                                RowLayout {
                                    anchors.fill: parent
                                    anchors.margins: 12
                                    spacing: 12

                                    Rectangle {
                                        Layout.preferredWidth: 8
                                        Layout.preferredHeight: 8
                                        radius: 4
                                        color: currentMapName === modelData ? "#3b82f6" : "transparent"
                                    }

                                    Text {
                                        Layout.fillWidth: true
                                        text: modelData
                                        font.pixelSize: 16
                                        font.bold: currentMapName === modelData
                                        color: currentMapName === modelData ? "#1e40af" : "#374151"
                                        verticalAlignment: Text.AlignVCenter
                                    }

                                    Button {
                                        Layout.preferredWidth: 80
                                        Layout.preferredHeight: 36
                                        text: "Chọn"
                                        font.pixelSize: 14
                                        font.bold: true

                                        background: Rectangle {
                                            radius: 6
                                            color: parent.pressed ? "#1d4ed8" : (parent.hovered ? "#3b82f6" : "#2563eb")
                                            border.color: parent.hovered ? "#1e40af" : "transparent"
                                            border.width: 1
                                        }

                                        contentItem: Text {
                                            text: parent.text
                                            font: parent.font
                                            color: "white"
                                            horizontalAlignment: Text.AlignHCenter
                                            verticalAlignment: Text.AlignVCenter
                                        }

                                        onClicked: {
                                            currentMapName = modelData
                                            root.mapSelected(modelData)
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // Right Panel - Map Information
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
                color: "#f9fafb"
                radius: 12
                border.color: "#e5e7eb"
                border.width: 1
                z: 1
            }

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 15
                spacing: 15
                z: 2

                // Title
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 10

                    Rectangle {
                        width: 36
                        height: 36
                        radius: 8
                        color: "#fce7f3"
                        
                        FontAwesome {
                            anchors.centerIn: parent
                            icon: "circle-info"
                            size: 20
                            color: "#db2777"
                        }
                    }

                    Text {
                        Layout.fillWidth: true
                        text: "Thông tin bản đồ"
                        font.pixelSize: 20
                        font.bold: true
                        color: "#1f2937"
                        wrapMode: Text.WordWrap
                    }
                }

                // Map Name
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 75
                    color: "#ffffff"
                    radius: 8
                    border.color: "#e5e7eb"
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 12
                        spacing: 8

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 8

                            Rectangle {
                                width: 28
                                height: 28
                                radius: 6
                                color: "#dbeafe"
                                
                                FontAwesome {
                                    anchors.centerIn: parent
                                    icon: "map"
                                    size: 16
                                    color: "#2563eb"
                                }
                            }

                            Text {
                                text: "Tên bản đồ"
                                font.pixelSize: 12
                                color: "#6b7280"
                            }
                        }

                        Text {
                            Layout.fillWidth: true
                            text: currentMapName
                            font.pixelSize: 16
                            font.bold: true
                            color: "#1f2937"
                            wrapMode: Text.WrapAnywhere
                            elide: Text.ElideRight
                        }
                    }
                }

                // Origin Coordinates
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 160
                    color: "#ffffff"
                    radius: 8
                    border.color: "#e5e7eb"
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 12
                        spacing: 10

                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 8

                            Rectangle {
                                width: 28
                                height: 28
                                radius: 6
                                color: "#fef3c7"
                                
                                FontAwesome {
                                    anchors.centerIn: parent
                                    icon: "location"
                                    size: 16
                                    color: "#d97706"
                                }
                            }

                            Text {
                                text: "Tọa độ gốc"
                                font.pixelSize: 12
                                color: "#6b7280"
                            }
                        }

                        // X coordinate
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 6

                            Text {
                                Layout.preferredWidth: 35
                                text: "X:"
                                font.pixelSize: 13
                                font.bold: true
                                color: "#374151"
                            }

                            Text {
                                Layout.fillWidth: true
                                text: originX.toFixed(3) + " m"
                                font.pixelSize: 14
                                color: "#1f2937"
                                elide: Text.ElideRight
                            }
                        }

                        // Y coordinate
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 6

                            Text {
                                Layout.preferredWidth: 35
                                text: "Y:"
                                font.pixelSize: 13
                                font.bold: true
                                color: "#374151"
                            }

                            Text {
                                Layout.fillWidth: true
                                text: originY.toFixed(3) + " m"
                                font.pixelSize: 14
                                color: "#1f2937"
                                elide: Text.ElideRight
                            }
                        }

                        // Theta coordinate
                        RowLayout {
                            Layout.fillWidth: true
                            spacing: 6

                            Text {
                                Layout.preferredWidth: 35
                                text: "θ:"
                                font.pixelSize: 13
                                font.bold: true
                                color: "#374151"
                            }

                            Text {
                                Layout.fillWidth: true
                                text: originTheta.toFixed(2) + "°"
                                font.pixelSize: 14
                                color: "#1f2937"
                                elide: Text.ElideRight
                            }
                        }
                    }
                }

                Item {
                    Layout.fillHeight: true
                }

                // Add Tag Button - Right aligned
                RowLayout {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 60

                    Item {
                        Layout.fillWidth: true
                    }

                    Item {
                        Layout.preferredWidth: 140
                        Layout.preferredHeight: 60

                        // Shadow
                        Rectangle {
                            anchors.fill: parent
                            anchors.topMargin: 4
                            anchors.leftMargin: 2
                            color: "#30000000"
                            radius: 12
                            z: 0
                        }

                        Button {
                            id: addTagButton
                            anchors.fill: parent
                            text: "Add Tag"
                            font.pixelSize: 16
                            font.bold: true

                            background: Rectangle {
                                radius: 12
                                color: addTagButton.pressed ? "#059669" : (addTagButton.hovered ? "#10b981" : "#34d399")
                                border.color: addTagButton.hovered ? "#047857" : "transparent"
                                border.width: 2

                                gradient: Gradient {
                                    GradientStop { position: 0.0; color: addTagButton.pressed ? "#059669" : (addTagButton.hovered ? "#10b981" : "#34d399") }
                                    GradientStop { position: 1.0; color: addTagButton.pressed ? "#047857" : (addTagButton.hovered ? "#059669" : "#10b981") }
                                }
                            }

                            contentItem: RowLayout {
                                spacing: 10
                                anchors.centerIn: parent

                                FontAwesome {
                                    icon: "circle-check"
                                    size: 20
                                    color: "white"
                                }

                                Text {
                                    text: addTagButton.text
                                    font: addTagButton.font
                                    color: "white"
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                }
                            }

                            onClicked: {
                                root.addTagClicked()
                            }
                        }
                    }
                }
            }
        }
    }
}