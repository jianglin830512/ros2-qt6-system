import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

RowLayout {
    id: root

    property string labelText: "选项:"
    property string leftText: "禁用"
    property string rightText: "启用"

    // 【反馈值】：true/false，控制反馈框显示什么文字
    property bool isCurrentOn: false

    // 【设定值】：true/false，控制开关状态
    property alias isSettingOn: control.checked

    spacing: 10

    // 1. 标签
    Label {
        text: root.labelText
        color: Theme.textColor
        font: Theme.defaultFont
        Layout.fillWidth: true
        Layout.alignment: Qt.AlignVCenter
    }

    // 2. 反馈值显示 (实心背景)
    Item{
        Layout.preferredWidth: 160
        Layout.preferredHeight: 60
        Rectangle {
            width: 100
            height: 40
            anchors.centerIn: parent
            color: Theme.highlightColor
            radius: 15

            Label {
                anchors.centerIn: parent
                // 根据 isCurrentOn 显示对应文字
                text: root.isCurrentOn ? root.rightText : root.leftText
                color: "white"
                font: Theme.defaultFont
            }
        }
    }

    // 3. 设定控制区域
    RowLayout {
        spacing: 5
        Label {
            text: root.leftText
            color: Theme.textColor
            font: Theme.smallLabelFont
        }

        Switch {
            id: control
            Layout.alignment: Qt.AlignVCenter
            // 样式保持之前的 Theme 风格...
            indicator: Rectangle {
                implicitWidth: 60
                implicitHeight: 30
                x: control.leftPadding
                y: parent.height / 2 - height / 2
                radius: 15
                color: "transparent"
                border.color: Theme.highlightColor
                border.width: 2
                Rectangle {
                    x: control.checked ? parent.width - width - 4 : 4
                    y: 4
                    width: 22
                    height: 22
                    radius: 11
                    color: control.checked ? Theme.titleColor : Theme.textColor
                    Behavior on x { NumberAnimation { duration: 200 } }
                }
            }
        }

        Label {
            text: root.rightText
            color: Theme.textColor
            font: Theme.smallLabelFont
        }
    }
}
