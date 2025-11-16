import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import Qt.labs.qmlmodels 1.0

Item {
    id: root
    
    signal resetClicked()
    signal clearClicked()
    
    // Properties để nhận dữ liệu từ C++ backend (robotData)
    property var dateArray: robotData ? robotData.logDateArray : ["14/11/25", "14/11/25", "14/11/25"]
    property var timeArray: robotData ? robotData.logTimeArray : ["15:30:12", "15:31:02", "15:32:47"]
    property var logArray: robotData ? robotData.logMessageArray : ["System started", "Connected to ROS 2", "Received feedback"]

    // Function để tạo rows từ các mảng
    function createRows() {
        var rows = []
        var maxLength = Math.max(dateArray.length, Math.max(timeArray.length, logArray.length))
        for (var i = 0; i < maxLength; i++) {
            rows.push({
                "date": i < dateArray.length ? dateArray[i] : "",
                "time": i < timeArray.length ? timeArray[i] : "",
                "log": i < logArray.length ? logArray[i] : ""
            })
        }
        return rows
    }

    // Cập nhật model khi các mảng thay đổi
    onDateArrayChanged: if (tableModel) tableModel.rows = createRows()
    onTimeArrayChanged: if (tableModel) tableModel.rows = createRows()
    onLogArrayChanged: if (tableModel) tableModel.rows = createRows()

    RowLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        // Log Table
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

            Rectangle {
                id: logTableContainer
                anchors.fill: parent
                color: "transparent"
                z: 2

            // Header row
            Rectangle {
                id: logHeader
                anchors.top: logTableContainer.top
                anchors.left: logTableContainer.left
                anchors.right: logTableContainer.right
                height: 36
                color: "#f9fafb"
                border.color: "#e5e7eb"

                Row {
                    anchors.fill: parent
                    spacing: 0

                    Rectangle {
                        width: 140
                        height: parent.height
                        color: "transparent"
                        border.color: "#e5e7eb"
                        Text {
                            anchors.centerIn: parent
                            text: "Date"
                            font.bold: true
                            font.pixelSize: 14
                        }
                    }

                    Rectangle {
                        width: 100
                        height: parent.height
                        color: "transparent"
                        border.color: "#e5e7eb"
                        Text {
                            anchors.centerIn: parent
                            text: "Time"
                            font.bold: true
                            font.pixelSize: 14
                        }
                    }

                    Rectangle {
                        width: parent.width - 240
                        height: parent.height
                        color: "transparent"
                        border.color: "#e5e7eb"
                        Text {
                            anchors.centerIn: parent
                            text: "History Log"
                            font.bold: true
                            font.pixelSize: 14
                        }
                    }
                }
            }

            // Table model + view
            TableView {
                id: logView
                anchors.top: logHeader.bottom
                anchors.left: logTableContainer.left
                anchors.right: logTableContainer.right
                anchors.bottom: logTableContainer.bottom
                clip: true
                columnSpacing: 1
                rowSpacing: 1

                model: TableModel {
                    id: tableModel
                    TableModelColumn { display: "date" }
                    TableModelColumn { display: "time" }
                    TableModelColumn { display: "log" }
                    rows: createRows()
                }

                columnWidthProvider: function(col) {
                    return col === 0 ? 140 : (col === 1 ? 100 : Math.max(0, width - 241));
                }

                rowHeightProvider: function(row) { return 32; }

                delegate: Rectangle {
                    implicitWidth: 100
                    implicitHeight: 32
                    border.width: 1
                    border.color: "#e5e7eb"
                    color: (row % 2 === 0) ? "#ffffff" : "#f9fafb"
                    Text {
                        text: display
                        anchors.centerIn: parent
                    }
                }
            }
            }
        }

        // Action Buttons
        Item {
            Layout.minimumWidth: 160
            Layout.maximumWidth: 160
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
                width: parent.width - 20
                z: 2

            // RESET Button
            Item {
                Layout.fillWidth: true
                Layout.preferredHeight: 80

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
                    id: resetButton
                    anchors.fill: parent
                    text: "RESET"
                    font.pixelSize: 16
                    font.bold: true

                    background: Rectangle {
                        radius: 12
                        color: resetButton.pressed ? "#ea580c" : (resetButton.hovered ? "#f97316" : "#fb923c")
                        border.color: resetButton.hovered ? "#ea580c" : "transparent"
                        border.width: 2

                        gradient: Gradient {
                            GradientStop { position: 0.0; color: resetButton.pressed ? "#ea580c" : (resetButton.hovered ? "#f97316" : "#fb923c") }
                            GradientStop { position: 1.0; color: resetButton.pressed ? "#c2410c" : (resetButton.hovered ? "#ea580c" : "#f97316") }
                        }
                    }

                    contentItem: RowLayout {
                        spacing: 10
                        anchors.centerIn: parent

                        FontAwesome {
                            icon: "arrow-rotate-right"
                            size: 20
                            color: "white"
                        }

                        Text {
                            text: resetButton.text
                            font: resetButton.font
                            color: "white"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                    
                    onClicked: {
                        root.resetClicked()
                    }
                }
            }

            // CLEAR Button
            Item {
                Layout.fillWidth: true
                Layout.preferredHeight: 80

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
                    id: clearButton
                    anchors.fill: parent
                    text: "CLEAR"
                    font.pixelSize: 16
                    font.bold: true

                    background: Rectangle {
                        radius: 12
                        color: clearButton.pressed ? "#dc2626" : (clearButton.hovered ? "#ef4444" : "#f87171")
                        border.color: clearButton.hovered ? "#dc2626" : "transparent"
                        border.width: 2

                        gradient: Gradient {
                            GradientStop { position: 0.0; color: clearButton.pressed ? "#dc2626" : (clearButton.hovered ? "#ef4444" : "#f87171") }
                            GradientStop { position: 1.0; color: clearButton.pressed ? "#991b1b" : (clearButton.hovered ? "#dc2626" : "#ef4444") }
                        }
                    }

                    contentItem: RowLayout {
                        spacing: 10
                        anchors.centerIn: parent

                        FontAwesome {
                            icon: "circle-stop"
                            size: 20
                            color: "white"
                        }

                        Text {
                            text: clearButton.text
                            font: clearButton.font
                            color: "white"
                            horizontalAlignment: Text.AlignHCenter
                            verticalAlignment: Text.AlignVCenter
                        }
                    }
                    
                    onClicked: {
                        root.clearClicked()
                    }
                }
            }
            }
        }
    }
}

