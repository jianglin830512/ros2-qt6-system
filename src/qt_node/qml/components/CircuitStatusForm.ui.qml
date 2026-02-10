import QtQuick
import QtQuick.Layouts
import qt.theme 1.0

Rectangle{
    color: "transparent"
    border.color: Theme.highlightColor
    border.width: 3
    radius: 8

    // --- Aliases for logic file ---
    property alias testLoop: testLoop
    property alias refLoop: refLoop
    property alias modeControl: modeControl
    property alias isModeBlocked: modeBlocker.visible

    RowLayout {
        anchors{
            top: parent.top
            left: parent.left
            right: parent.right
            bottom: modeControl.top
            margins: 10
        }
        spacing: Theme.subSpacing
        LoopStatus {
            id: testLoop
        }
        LoopStatus {
            id: refLoop
        }
    }

    ModeControl {
        id: modeControl
        anchors{
            left: parent.left
            right: parent.right
            bottom: parent.bottom
            bottomMargin: 10
        }
        // height: 60
    }

    InputBlocker {
        id: modeBlocker
        anchors.fill: modeControl // 这一行很关键，它只遮住下方的控制条
        anchors{                  // 稍微扩大一点点遮挡范围，视觉上更严密
            topMargin: -5
            bottomMargin: -5
            leftMargin: 10
            rightMargin: 10
        }
        statusText: "" // 同样留空，仅变暗
    }
}
