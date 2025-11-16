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
        GradientStop { position: 0.0; color: "#9c3618" }
        GradientStop { position: 1.0; color: "#7e270b" }
    }

    property int currentPageIndex: 0
    signal pageChanged(int pageIndex, string pageName)

    RowLayout {
        anchors.fill: parent
        spacing: 0

        Button {
            id: manualBtn
            Layout.minimumWidth: 204
            Layout.maximumWidth: 204
            Layout.fillHeight: true
            flat: true
            text: ""
            
            contentItem: FontAwesome {
                icon: "gamepad"
                size: 40
                color: currentPageIndex === 1 ? "#7e270b" : "white"
                anchors.centerIn: parent
            }
            
            scale: manualBtn.pressed ? 0.95 : (manualBtn.hovered ? 1.05 : 1.0)
            
            Behavior on scale {
                NumberAnimation { duration: 150 }
            }

            background: Rectangle {
                color: {
                    if (currentPageIndex === 1) return "#fbbf24"
                    if (parent.hovered) return "#99ffffff"
                    return "transparent"
                }
                
                Behavior on color {
                    ColorAnimation { duration: 200 }
                }
                
                // Active indicator
                Rectangle {
                    visible: currentPageIndex === 1
                    anchors.bottom: parent.bottom
                    anchors.horizontalCenter: parent.horizontalCenter
                    width: parent.width * 0.6
                    height: 4
                    radius: 2
                    color: "#7e270b"
                }
            }

            onClicked: root.pageChanged(1, "MANUAL")
        }

        Button {
            id: settingsBtn
            Layout.minimumWidth: 204
            Layout.maximumWidth: 204
            Layout.fillHeight: true
            flat: true
            text: ""
            
            contentItem: FontAwesome {
                icon: "gear"
                size: 40
                color: currentPageIndex === 2 ? "#7e270b" : "white"
                anchors.centerIn: parent
            }
            
            scale: settingsBtn.pressed ? 0.95 : (settingsBtn.hovered ? 1.05 : 1.0)
            
            Behavior on scale {
                NumberAnimation { duration: 150 }
            }

            background: Rectangle {
                color: {
                    if (currentPageIndex === 2) return "#fbbf24"
                    if (parent.hovered) return "#99ffffff"
                    return "transparent"
                }
                
                Behavior on color {
                    ColorAnimation { duration: 200 }
                }
                
                Rectangle {
                    visible: currentPageIndex === 2
                    anchors.bottom: parent.bottom
                    anchors.horizontalCenter: parent.horizontalCenter
                    width: parent.width * 0.6
                    height: 4
                    radius: 2
                    color: "#7e270b"
                }
            }

            onClicked: root.pageChanged(2, "SETTINGS")
        }

        Button {
            id: homeBtn
            Layout.minimumWidth: 204
            Layout.maximumWidth: 204
            Layout.fillHeight: true
            flat: true
            text: ""
            
            contentItem: FontAwesome {
                icon: "home"
                size: 40
                color: currentPageIndex === 0 ? "#7e270b" : "white"
                anchors.centerIn: parent
            }
            
            scale: homeBtn.pressed ? 0.95 : (homeBtn.hovered ? 1.05 : 1.0)
            
            Behavior on scale {
                NumberAnimation { duration: 150 }
            }

            background: Rectangle {
                color: {
                    if (currentPageIndex === 0) return "#fbbf24"
                    if (parent.hovered) return "#99ffffff"
                    return "transparent"
                }
                
                Behavior on color {
                    ColorAnimation { duration: 200 }
                }
                
                Rectangle {
                    visible: currentPageIndex === 0
                    anchors.bottom: parent.bottom
                    anchors.horizontalCenter: parent.horizontalCenter
                    width: parent.width * 0.6
                    height: 4
                    radius: 2
                    color: "#7e270b"
                }
            }

            onClicked: root.pageChanged(0, "HOME")
        }

        Button {
            id: missionBtn
            Layout.minimumWidth: 204
            Layout.maximumWidth: 204
            Layout.fillHeight: true
            flat: true
            text: ""
            
            contentItem: FontAwesome {
                icon: "map"
                size: 45
                color: currentPageIndex === 3 ? "#7e270b" : "white"
                anchors.centerIn: parent
            }
            
            scale: missionBtn.pressed ? 0.95 : (missionBtn.hovered ? 1.05 : 1.0)
            
            Behavior on scale {
                NumberAnimation { duration: 150 }
            }

            background: Rectangle {
                color: {
                    if (currentPageIndex === 3) return "#fbbf24"
                    if (parent.hovered) return "#99ffffff"
                    return "transparent"
                }
                
                Behavior on color {
                    ColorAnimation { duration: 200 }
                }
                
                Rectangle {
                    visible: currentPageIndex === 3
                    anchors.bottom: parent.bottom
                    anchors.horizontalCenter: parent.horizontalCenter
                    width: parent.width * 0.6
                    height: 4
                    radius: 2
                    color: "#7e270b"
                }
            }

            onClicked: root.pageChanged(3, "MISSION")
        }

        Button {
            id: infoBtn
            Layout.minimumWidth: 204
            Layout.maximumWidth: 204
            Layout.fillHeight: true
            flat: true
            text: ""
            
            contentItem: FontAwesome {
                icon: "circle-info"
                size: 45
                color: currentPageIndex === 4 ? "#7e270b" : "white"
                anchors.centerIn: parent
            }
            
            scale: infoBtn.pressed ? 0.95 : (infoBtn.hovered ? 1.05 : 1.0)
            
            Behavior on scale {
                NumberAnimation { duration: 150 }
            }

            background: Rectangle {
                color: {
                    if (currentPageIndex === 4) return "#fbbf24"
                    if (parent.hovered) return "#99ffffff"
                    return "transparent"
                }
                
                Behavior on color {
                    ColorAnimation { duration: 200 }
                }
                
                Rectangle {
                    visible: currentPageIndex === 4
                    anchors.bottom: parent.bottom
                    anchors.horizontalCenter: parent.horizontalCenter
                    width: parent.width * 0.6
                    height: 4
                    radius: 2
                    color: "#7e270b"
                }
            }

            onClicked: root.pageChanged(4, "INFO")
        }
    }
}

