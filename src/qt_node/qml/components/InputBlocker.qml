import QtQuick

Item {
    id: root
    // 默认隐藏
    visible: false

    // 遮罩颜色
    property color overlayColor: "#99000000"

    // 提示文字
    property string statusText: "未启用"

    // 【核心修复】显式定义 radius 属性，供外部调用
    property real radius: 0

    anchors.fill: parent
    z: 999

    Rectangle {
        anchors.fill: parent
        color: root.overlayColor

        // 【核心修复】内部 Rectangle 使用根组件传入的 radius
        radius: root.radius

        Text {
            anchors.centerIn: parent
            text: root.statusText
            color: "#AAAAAA"
            font.pixelSize: 24
            font.bold: true
            visible: root.statusText !== ""
            opacity: 0.7
        }
    }

    MouseArea {
        anchors.fill: parent
        hoverEnabled: true
        preventStealing: true
        onPressed: (mouse) => mouse.accepted = true
        onReleased: (mouse) => mouse.accepted = true
        onClicked: (mouse) => mouse.accepted = true
        onWheel: (wheel) => wheel.accepted = true
    }
}
