import QtQuick 2.15

Item {
    id: root
    
    property string icon: ""
    property int size: 16
    property color color: "black"
    
    // FontAwesome 6 Solid Icon Unicode
    readonly property var icons: ({
        "location": "\uf3c5",
        "bolt": "\uf0e7",
        "map": "\uf279",
        "bullseye": "\uf140",
        "signal": "\uf012",
        "microchip": "\uf2db",
        "memory": "\uf538",
        "clock": "\uf017",
        "gauge": "\uf624",
        "route": "\uf4d7",
        "crosshairs": "\uf05b",
        "server": "\uf233",
        "battery-full": "\uf240",
        "circle-check": "\uf058",
        "triangle-exclamation": "\uf071",
        "thermometer": "\uf2c7",
        "hard-drive": "\uf0a0",
        "circle-stop": "\uf28d",
        "play": "\uf04b",
        "home": "\uf015",
        "gear": "\uf013",
        "gamepad": "\uf11b",
        "clipboard-list": "\uf46d",
        "circle-info": "\uf05a",
        "bell": "\uf0f3",
        "arrow-up": "\uf062",
        "arrow-down": "\uf063",
        "arrow-left": "\uf060",
        "arrow-right": "\uf061",
        "arrow-rotate-right": "\uf01e",
        "arrows-up-down": "\uf07d",
        "volume-high": "\uf028"
    })
    
    width: iconText.width
    height: iconText.height
    
    FontLoader {
        id: fontAwesome
        source: "qrc:/fonts/fontawesome-solid.ttf"
    }
    
    Text {
        id: iconText
        font.family: fontAwesome.name
        font.pixelSize: root.size
        color: root.color
        text: icons[root.icon] || ""
        anchors.centerIn: parent
    }
}

