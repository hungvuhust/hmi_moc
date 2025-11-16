import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15

ApplicationWindow {
    id: window
    width: 1024
    height: 600
    minimumWidth: 1024
    minimumHeight: 600
    maximumWidth: 1024
    maximumHeight: 600
    title: "Robot HMI"
    visible: true

    property int currentPageIndex: 0
    property string currentPageTitle: "HOME"
    


    ColumnLayout {
        anchors.fill: parent
        spacing: 0

        // Header
        Header {
            id: header
            operationMode: robotData.operationMode
            batteryLevel: robotData.batteryLevel
        }

        // Main Content
        StackLayout {
            id: stackedWidget
            Layout.fillWidth: true
            Layout.minimumHeight: 480
            Layout.maximumHeight: 480
            currentIndex: currentPageIndex

            HomePage {
                id: homePage
                
                // Navigation data
                posX: robotData.posX
                posY: robotData.posY
                theta: robotData.theta
                speed: robotData.speed
                mapName: robotData.mapName
                destination: robotData.destination
                localizationScore: robotData.localizationScore
                
                // System data
                cpuUsage: deviceStatus.cpuUsage
                ramUsage: deviceStatus.ramUsage
                temperature: deviceStatus.temperature
                memUsage: deviceStatus.memUsage
                timeUsed: deviceStatus.timeUsed
                
                // Status data
                batteryLevel: robotData.batteryLevel
                robotState: robotData.robotState
                
                // Device errors
                emgFrontError: robotData.emgFrontError
                emgRearError: robotData.emgRearError
                motorLeftError: robotData.motorLeftError
                motorRightError: robotData.motorRightError
                lidarFrontError: robotData.lidarFrontError
                lidarRearError: robotData.lidarRearError
                
                // Button signals
                onStartClicked: {
                    if (robotData && typeof robotData.start === "function") {
                        robotData.start()
                    }
                }
                onStopClicked: {
                    if (robotData && typeof robotData.stop === "function") {
                        robotData.stop()
                    }
                }
                onRelocationClicked: {
                    if (robotData && typeof robotData.relocation === "function") {
                        robotData.relocation()
                    }
                }
            }

            ManualPage {
                id: manualPage
                onVelocityChanged: function(linear, angular) {
                    if (robotData && typeof robotData.setSpeed === "function") {
                        robotData.setSpeed(linear, angular)
                    }
                }
            }

            SettingsPage {
                id: settingsPage
                onVolumeChanged: function(volume) {
                    if (deviceStatus && typeof deviceStatus.setVolume === "function") {
                        deviceStatus.setVolume(volume)
                    }
                }
                onMaxVelocityChanged: function(velocity) {
                    if (robotData && typeof robotData.setMaxVelocity === "function") {
                        robotData.setMaxVelocity(velocity)
                    }
                }
            }

            MapPanel {
                id: mapPanel
                mapList: robotData.mapList
                currentMapName: robotData.currentMapName
                originX: robotData.originX
                originY: robotData.originY
                originTheta: robotData.originTheta
                
                onMapSelected: function(mapName) {
                    if (robotData && typeof robotData.selectMap === "function") {
                        robotData.selectMap(mapName)
                    }
                }
                onAddTagClicked: {
                    if (robotData && typeof robotData.addTag === "function") {
                        robotData.addTag()
                    }
                }
            }

            InfoPage {
                id: infoPage
                onResetClicked: {
                    if (robotData && typeof robotData.reset === "function") {
                        robotData.reset()
                    }
                }
                onClearClicked: {
                    if (robotData && typeof robotData.clearLog === "function") {
                        robotData.clearLog()
                    }
                }
            }
        }

        // Navigation Bar
        NavigationBar {
            id: navBar
            currentPageIndex: window.currentPageIndex
            onPageChanged: function(pageIndex, pageName) {
                window.currentPageIndex = pageIndex;
                window.currentPageTitle = pageName;
            }
        }
    }
}
