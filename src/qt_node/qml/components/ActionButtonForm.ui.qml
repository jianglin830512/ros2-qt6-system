import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

// 使用 Control 作为根，因为它提供了 enabled, pressed, hovered 等基础状态
Button {
    id: root
    implicitWidth: 120
    implicitHeight: 45

    // --- 通过别名将内部元素暴露给逻辑层 ---
    property string labelText: "button"
    property bool indicatorOn: false
    property color colorWhenOn: "red"

    // --- 背景 ---
    // 根据按钮的状态（禁用、按下）改变外观
    background: Rectangle {
        anchors.fill: parent
        radius: 8
        // 按钮按下时颜色变深，提供视觉反馈
        color: root.pressed ? Qt.darker(Theme.buttonSelectedGradientEnd, 1.2) : Theme.buttonSelectedGradientEnd
        border.color: Theme.highlightColor
        border.width: 1
        // 按钮禁用时，整个控件变半透明
        opacity: root.enabled ? 1.0 : 0.5

        Item{
            id: textArea
            anchors{
                left: parent.left
                top: parent.top
                bottom: parent.bottom
            }
            width: parent.width * 0.7
            Label {
                id: textLabel
                anchors.centerIn: parent
                anchors.leftMargin: 5
                text: labelText
                font: Theme.buttonFont
                // 按钮禁用时，文本颜色变灰
                color: root.enabled ? Theme.buttonSelectedTextColor : Qt.darker(Theme.textColor)
            }
        }

        // --- 指示灯 ---
        Item{
            anchors{
                left: textArea.right
                top: parent.top
                bottom: parent.bottom
                right: parent.right
            }
            Rectangle {
                id: indicator
                // --- 指示灯的公共API属性 ---
                anchors{
                    fill: parent
                    leftMargin: 5
                    rightMargin: 10
                    topMargin: 5
                    bottomMargin: 5
                }
                radius: width / 2
                border.color: "#202020"
                border.width: 2

                // 根据 isOn 属性决定显示预设颜色还是黑色
                color: indicatorOn ? colorWhenOn : "black"
            }
        }
    }
}
