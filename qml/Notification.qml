import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    anchors.fill: parent
    
    // Queue for notifications
    property var notificationQueue: []
    property int maxVisible: 3
    
    function show(message, type) {
        // type: "success", "error", "info", "warning"
        var notification = {
            message: message,
            type: type,
            timestamp: Date.now()
        }
        
        notificationQueue.push(notification)
        if (repeater.count < maxVisible) {
            repeater.model = notificationQueue.length
        }
    }
    
    Column {
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.topMargin: 80  // Tăng lên để không bị Header che
        spacing: 10
        z: 9999
        
        width: 420
        
        Repeater {
            id: repeater
            model: 0
            
            Rectangle {
                id: notificationRect
                width: 400
                height: 60
                radius: 10
                color: getBackgroundColor()
                border.color: getBorderColor()
                border.width: 2
                opacity: 0
                
                property var notificationData: index < notificationQueue.length ? notificationQueue[index] : null
                
                function getBackgroundColor() {
                    if (!notificationData) return "white"
                    switch(notificationData.type) {
                        case "success": return "#d1fae5"
                        case "error": return "#fee2e2"
                        case "warning": return "#fef3c7"
                        case "info": return "#dbeafe"
                        default: return "white"
                    }
                }
                
                function getBorderColor() {
                    if (!notificationData) return "#e5e7eb"
                    switch(notificationData.type) {
                        case "success": return "#10b981"
                        case "error": return "#ef4444"
                        case "warning": return "#f59e0b"
                        case "info": return "#3b82f6"
                        default: return "#e5e7eb"
                    }
                }
                
                function getIconName() {
                    if (!notificationData) return "circle-info"
                    switch(notificationData.type) {
                        case "success": return "circle-check"
                        case "error": return "circle-xmark"
                        case "warning": return "triangle-exclamation"
                        case "info": return "circle-info"
                        default: return "circle-info"
                    }
                }
                
                function getTextColor() {
                    if (!notificationData) return "#374151"
                    switch(notificationData.type) {
                        case "success": return "#065f46"
                        case "error": return "#991b1b"
                        case "warning": return "#92400e"
                        case "info": return "#1e40af"
                        default: return "#374151"
                    }
                }
                
                // Shadow
                Rectangle {
                    anchors.fill: parent
                    anchors.topMargin: 3
                    anchors.leftMargin: 3
                    radius: parent.radius
                    color: "#40000000"
                    z: -1
                }
                
                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 12
                    spacing: 12
                    
                    FontAwesome {
                        icon: parent.parent.getIconName()
                        size: 24
                        color: parent.parent.getBorderColor()
                    }
                    
                    Text {
                        Layout.fillWidth: true
                        text: notificationData ? notificationData.message : ""
                        font.pixelSize: 14
                        font.bold: true
                        color: parent.parent.getTextColor()
                        wrapMode: Text.WordWrap
                        elide: Text.ElideRight
                        maximumLineCount: 2
                    }
                    
                    Button {
                        Layout.preferredWidth: 24
                        Layout.preferredHeight: 24
                        flat: true
                        
                        contentItem: FontAwesome {
                            icon: "xmark"
                            size: 16
                            color: "#6b7280"
                            anchors.centerIn: parent
                        }
                        
                        background: Rectangle {
                            color: parent.hovered ? "#f3f4f6" : "transparent"
                            radius: 4
                        }
                        
                        onClicked: hideAnimation.start()
                    }
                }
                
                // Show animation
                Component.onCompleted: {
                    showAnimation.start()
                    autoHideTimer.start()
                }
                
                SequentialAnimation {
                    id: showAnimation
                    NumberAnimation {
                        target: notificationRect
                        property: "opacity"
                        from: 0
                        to: 1
                        duration: 300
                        easing.type: Easing.OutCubic
                    }
                }
                
                SequentialAnimation {
                    id: hideAnimation
                    NumberAnimation {
                        target: notificationRect
                        property: "opacity"
                        to: 0
                        duration: 200
                        easing.type: Easing.InCubic
                    }
                    ScriptAction {
                        script: {
                            notificationQueue.shift()
                            repeater.model = notificationQueue.length
                        }
                    }
                }
                
                Timer {
                    id: autoHideTimer
                    interval: 3000
                    onTriggered: hideAnimation.start()
                }
            }
        }
    }
}

