import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

Button {
    id: root
    implicitWidth: 120
    implicitHeight: 45

    property string labelText: "button"
    property bool indicatorOn: false
    property color colorWhenOn: "red"

    // 【新增】操作按钮的硬核按下反馈 (下陷 4%)
    scale: root.pressed ? 0.96 : 1.0
    Behavior on scale { NumberAnimation { duration: 50 } }

    background: Rectangle {
        anchors.fill: parent
        radius: 8

        // 【核心反色逻辑】
        // 按下瞬间，背景填充深蓝色；禁用时使用浅灰背景；平时是背景白色
        color: root.pressed ? Theme.buttonSelectedGradientEnd : (!root.enabled ? Theme.buttonHoverColor : Theme.buttonBackgroundColor)

        // 边框：按下时边框与背景同色融为一体，禁用时使用灰色边框；平时是蓝色边框
        border.color: root.pressed ? Theme.buttonSelectedGradientEnd : (!root.enabled ? Theme.buttonBorderColor : Theme.highlightColor)
        border.width: 2

        Item {
            id: textArea
            anchors {
                left: parent.left; top: parent.top; bottom: parent.bottom
            }
            width: parent.width * 0.7

            Label {
                id: textLabel
                anchors.centerIn: parent
                anchors.leftMargin: 5
                text: labelText
                font: Theme.buttonFont

                // 【文本反色】禁用时变灰；按下时文字强制纯白；平时为主色调蓝
                color: !root.enabled ? Theme.statusDisabledColor :
                                       (root.pressed ? "#FFFFFF" : Theme.titleColor)
            }
        }

        // --- 指示灯区域保持不变 ---
        Item {
            anchors {
                left: textArea.right; top: parent.top; bottom: parent.bottom; right: parent.right
            }
            Rectangle {
                id: indicator
                anchors {
                    fill: parent; leftMargin: 5; rightMargin: 10; topMargin: 5; bottomMargin: 5
                }
                radius: width / 2
                border.color: Theme.indicatorBorderColor
                border.width: 2
                color: indicatorOn ? colorWhenOn : Theme.indicatorOffColor
            }
        }
    }
}
