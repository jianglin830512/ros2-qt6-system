import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

RowLayout {
    id: root
    property int loopId: 1
    property bool isRunning: false // 用于控制指示灯状态

    // 信号
    signal startClicked()
    signal stopClicked()

    spacing: 20

    // 启动按钮
    StyledButton {
        Layout.fillWidth: true
        Layout.preferredHeight: 40
        text: "启动"

        // 指示灯 (绿色代表运行)
        Rectangle {
            width: 20; height: 20
            radius: 10
            color: root.isRunning ? "lime" : "#333"
            border.color: "black"
            anchors.right: parent.right
            anchors.rightMargin: 15
            anchors.verticalCenter: parent.verticalCenter
        }
        onClicked: root.startClicked()
    }

    // 停止按钮
    StyledButton {
        Layout.fillWidth: true
        Layout.preferredHeight: 40
        text: "停止"

        // 指示灯 (红色代表停止/待机)
        Rectangle {
            width: 20; height: 20
            radius: 10
            color: !root.isRunning ? "red" : "#333"
            border.color: "black"
            anchors.right: parent.right
            anchors.rightMargin: 15
            anchors.verticalCenter: parent.verticalCenter
        }
        onClicked: root.stopClicked()
    }
}
