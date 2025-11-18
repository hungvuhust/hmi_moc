import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

Item {
    id: root
    
    // Properties for robot control
    property real linearVelocity: 0.0  // m/s
    property real angularVelocity: 0.0  // rad/s
    property real maxLinearVelocity: 0.1  // m/s
    property real maxAngularVelocity: 0.1  // rad/s
    property bool isJoystickPressed: false  // Trạng thái joystick đang được nhấn
    
    signal velocityChanged(real linear, real angular)
    
    // Timer để publish velocity liên tục khi joystick đang được nhấn
    Timer {
        id: velocityPublishTimer
        interval: 100  // Publish mỗi 100ms
        repeat: true
        running: root.isJoystickPressed
        onTriggered: {
            if (root.isJoystickPressed) {
                root.velocityChanged(root.linearVelocity, root.angularVelocity)
            }
        }
    }
    
    Rectangle {
        anchors.fill: parent
        gradient: Gradient {
            GradientStop { position: 0.0; color: "#f8f9fa" }
            GradientStop { position: 1.0; color: "#e9ecef" }
        }
    }
    
    RowLayout {
        anchors.fill: parent
        anchors.margins: 15
        spacing: 15
        
        // Left Panel - Joystick Control (Larger)
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumWidth: 600
            radius: 20
            color: "white"
            border.color: "#dee2e6"
            border.width: 2
            
            // Shadow effect
            Rectangle {
                anchors.fill: parent
                anchors.topMargin: 4
                anchors.leftMargin: 4
                radius: parent.radius
                color: "#20000000"
                z: -1
            }
            
            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 25
                spacing: 15
                
                // Title
                Text {
                    Layout.alignment: Qt.AlignHCenter
                    text: "Điều khiển Robot"
                    font.pixelSize: 22
                    font.bold: true
                    color: "#7e270b"
                }
                
                // Joystick Area
                Item {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.minimumHeight: 280
                    
                    // Joystick Container
                    Rectangle {
                        id: joystickContainer
                        width: Math.min(parent.width * 0.70, parent.height * 0.70)
                        height: width
                        anchors.centerIn: parent
                        radius: width / 2
                        color: "#f8f9fa"
                        border.color: "#dee2e6"
                        border.width: 3
                        
                        // Grid lines for reference
                        Repeater {
                            model: 8
                            Rectangle {
                                width: joystickContainer.width
                                height: 1.5
                                anchors.centerIn: parent
                                rotation: index * 22.5
                                color: "#e9ecef"
                                opacity: 0.4
                            }
                        }
                        
                        // Center crosshair
                        Rectangle {
                            width: 2
                            height: joystickContainer.height * 0.4
                            anchors.centerIn: parent
                            color: "#ced4da"
                            opacity: 0.6
                        }
                        Rectangle {
                            width: joystickContainer.width * 0.4
                            height: 2
                            anchors.centerIn: parent
                            color: "#ced4da"
                            opacity: 0.6
                        }
                        
                        // Center circle
                        Rectangle {
                            width: 16
                            height: 16
                            radius: 8
                            anchors.centerIn: parent
                            color: "#ced4da"
                            opacity: 0.5
                        }
                        
                        // Joystick handle
                        Rectangle {
                            id: joystickHandle
                            width: 80
                            height: 80
                            radius: width / 2
                            x: joystickContainer.width / 2 - width / 2
                            y: joystickContainer.height / 2 - height / 2
                            
                            gradient: Gradient {
                                GradientStop { position: 0.0; color: "#7e270b" }
                                GradientStop { position: 1.0; color: "#9c3618" }
                            }
                            
                            border.color: "#5a1a0a"
                            border.width: 2
                            
                            // Inner circle
                            Rectangle {
                                anchors.centerIn: parent
                                width: parent.width * 0.5
                                height: parent.height * 0.5
                                radius: width / 2
                                color: "#ffffff"
                                opacity: 0.4
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
                        }
                        
                        // Drag area - placed outside handle
                        MouseArea {
                            id: joystickMouseArea
                            anchors.fill: joystickContainer
                            z: 1
                            
                            property real centerX: joystickContainer.width / 2
                            property real centerY: joystickContainer.height / 2
                            property real maxDistance: (joystickContainer.width - joystickHandle.width) / 2
                            
                            onPressed: {
                                // Stop any ongoing animation
                                returnToCenter.stop()
                                root.isJoystickPressed = true
                                updateJoystickPosition(mouseX, mouseY)
                            }
                            
                            onPositionChanged: {
                                if (pressed) {
                                    updateJoystickPosition(mouseX, mouseY)
                                }
                            }
                            
                            onReleased: {
                                root.isJoystickPressed = false
                                // Publish stop command immediately
                                root.linearVelocity = 0.0
                                root.angularVelocity = 0.0
                                root.velocityChanged(0.0, 0.0)
                                // Return to center with animation
                                returnToCenter.start()
                            }
                            
                            function updateJoystickPosition(x, y) {
                                var dx = x - centerX
                                var dy = y - centerY
                                var distance = Math.sqrt(dx * dx + dy * dy)
                                
                                if (distance > maxDistance) {
                                    dx = dx * maxDistance / distance
                                    dy = dy * maxDistance / distance
                                    distance = maxDistance
                                }
                                
                                // Update handle position directly (no animation during drag)
                                joystickHandle.x = centerX + dx - joystickHandle.width / 2
                                joystickHandle.y = centerY + dy - joystickHandle.height / 2
                                
                                // Calculate velocities
                                // Y axis (up/down): linear velocity (up = negative, down = positive)
                                // X axis (left/right): angular velocity (right = positive, left = negative)
                                var normalizedLinear = -dy / maxDistance  // Up = negative (backward), Down = positive (forward)
                                var normalizedAngular = dx / maxDistance  // Right = positive, Left = negative
                                
                                root.linearVelocity = -normalizedLinear * root.maxLinearVelocity
                                root.angularVelocity = -normalizedAngular * root.maxAngularVelocity
                                
                                // Timer sẽ publish velocity liên tục
                            }
                        }
                        
                        // Return to center animation
                        ParallelAnimation {
                            id: returnToCenter
                            running: false
                            
                            NumberAnimation {
                                target: joystickHandle
                                property: "x"
                                to: joystickContainer.width / 2 - joystickHandle.width / 2
                                duration: 200
                                easing.type: Easing.OutCubic
                            }
                            NumberAnimation {
                                target: joystickHandle
                                property: "y"
                                to: joystickContainer.height / 2 - joystickHandle.height / 2
                                duration: 200
                                easing.type: Easing.OutCubic
                            }
                        }
                        
                    }
                }
                
                // Velocity Display - Compact and visible
                RowLayout {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 75
                    spacing: 10
                    
                    // Linear Velocity
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        radius: 10
                        color: "#ffffff"
                        border.color: "#339af0"
                        border.width: 2
                        
                        // Shadow
                        Rectangle {
                            anchors.fill: parent
                            anchors.topMargin: 2
                            anchors.leftMargin: 2
                            radius: parent.radius
                            color: "#20000000"
                            z: -1
                        }
                        
                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 8
                            spacing: 4
                            
                            RowLayout {
                                Layout.fillWidth: true
                                
                                FontAwesome {
                                    icon: "arrows-up-down"
                                    size: 16
                                    color: "#1864ab"
                                }
                                
                                Text {
                                    text: "Tốc độ dài"
                                    font.pixelSize: 12
                                    font.bold: true
                                    color: "#1864ab"
                                }
                            }
                            
                            Text {
                                Layout.fillWidth: true
                                text: root.linearVelocity.toFixed(2) + " m/s"
                                font.pixelSize: 24
                                font.bold: true
                                color: "#1864ab"
                                horizontalAlignment: Text.AlignHCenter
                            }
                            
                            // Progress bar
                            Rectangle {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 5
                                radius: 2
                                color: "#d0ebff"
                                
                                Rectangle {
                                    width: parent.width * Math.abs(root.linearVelocity) / root.maxLinearVelocity
                                    height: parent.height
                                    radius: 2
                                    color: root.linearVelocity >= 0 ? "#339af0" : "#fa5252"
                                    
                                    Behavior on width {
                                        NumberAnimation { duration: 100 }
                                    }
                                }
                            }
                        }
                    }
                    
                    // Angular Velocity
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        radius: 10
                        color: "#ffffff"
                        border.color: "#ffc107"
                        border.width: 2
                        
                        // Shadow
                        Rectangle {
                            anchors.fill: parent
                            anchors.topMargin: 2
                            anchors.leftMargin: 2
                            radius: parent.radius
                            color: "#20000000"
                            z: -1
                        }
                        
                        ColumnLayout {
                            anchors.fill: parent
                            anchors.margins: 8
                            spacing: 4
                            
                            RowLayout {
                                Layout.fillWidth: true
                                
                                FontAwesome {
                                    icon: "arrow-rotate-right"
                                    size: 16
                                    color: "#b45309"
                                }
                                
                                Text {
                                    text: "Tốc độ góc"
                                    font.pixelSize: 12
                                    font.bold: true
                                    color: "#b45309"
                                }
                            }
                            
                            Text {
                                Layout.fillWidth: true
                                text: root.angularVelocity.toFixed(2) + " rad/s"
                                font.pixelSize: 24
                                font.bold: true
                                color: "#b45309"
                                horizontalAlignment: Text.AlignHCenter
                            }
                            
                            // Progress bar
                            Rectangle {
                                Layout.fillWidth: true
                                Layout.preferredHeight: 5
                                radius: 2
                                color: "#fff9db"
                                
                                Rectangle {
                                    width: parent.width * Math.abs(root.angularVelocity) / root.maxAngularVelocity
                                    height: parent.height
                                    radius: 2
                                    color: root.angularVelocity >= 0 ? "#ffc107" : "#fa5252"
                                    
                                    Behavior on width {
                                        NumberAnimation { duration: 100 }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        
        // Right Panel - Additional Controls (Narrower)
        ColumnLayout {
            Layout.preferredWidth: 320
            Layout.fillHeight: true
            spacing: 12
            
            // Max Speed Settings
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 200
                radius: 15
                color: "white"
                border.color: "#dee2e6"
                border.width: 2
                
                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 18
                    spacing: 12
                    
                    Text {
                        Layout.fillWidth: true
                        text: "Cài đặt tốc độ tối đa"
                        font.pixelSize: 15
                        font.bold: true
                        color: "#7e270b"
                    }
                    
                    // Max Linear Velocity
                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 6
                        
                        RowLayout {
                            Layout.fillWidth: true
                            
                            FontAwesome {
                                icon: "arrows-up-down"
                                size: 15
                                color: "#495057"
                            }
                            
                            Text {
                                text: "Tốc độ dài:"
                                font.pixelSize: 12
                                color: "#495057"
                            }
                            
                            Text {
                                Layout.fillWidth: true
                                text: root.maxLinearVelocity.toFixed(2) + " m/s"
                                font.pixelSize: 12
                                font.bold: true
                                color: "#7e270b"
                                horizontalAlignment: Text.AlignRight
                            }
                        }
                        
                        Slider {
                            id: maxLinearSlider
                            Layout.fillWidth: true
                            from: 0.1
                            to: 0.3
                            value: root.maxLinearVelocity
                            stepSize: 0.05
                            
                            onValueChanged: {
                                root.maxLinearVelocity = value
                            }
                        }
                    }
                    
                    // Max Angular Velocity
                    ColumnLayout {
                        Layout.fillWidth: true
                        spacing: 6
                        
                        RowLayout {
                            Layout.fillWidth: true
                            
                            FontAwesome {
                                icon: "arrow-rotate-right"
                                size: 15
                                color: "#495057"
                            }
                            
                            Text {
                                text: "Tốc độ góc:"
                                font.pixelSize: 12
                                color: "#495057"
                            }
                            
                            Text {
                                Layout.fillWidth: true
                                text: root.maxAngularVelocity.toFixed(2) + " rad/s"
                                font.pixelSize: 12
                                font.bold: true
                                color: "#7e270b"
                                horizontalAlignment: Text.AlignRight
                            }
                        }
                        
                        Slider {
                            id: maxAngularSlider
                            Layout.fillWidth: true
                            from: 0.1
                            to: 0.3
                            value: root.maxAngularVelocity
                            stepSize: 0.05
                            
                            onValueChanged: {
                                root.maxAngularVelocity = value
                            }
                        }
                    }
                }
            }
            
        }
    }
}
