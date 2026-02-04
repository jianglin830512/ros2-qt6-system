import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

RowLayout {
    id: root

    property string labelText: "参数名:"
    property string unitText: ""
    property string currentValue: "0"
    property alias settingValue: inputField.text
    property var inputValidator: IntValidator {}

    spacing: 10

    // 【新增】左侧弹性占位符 (把内容往中间挤)
    // Item { Layout.fillWidth: true }

    // 1. 标签 (固定宽度，右对齐，确保上下行对齐)
    Label {
        text: root.labelText
        color: Theme.textColor
        font: Theme.defaultFont
        Layout.alignment: Qt.AlignVCenter
        // 【修改】固定宽度，不再填充
        Layout.preferredWidth: 120
        horizontalAlignment: Text.AlignRight
    }

    // 2. 反馈值显示
    Rectangle {
        Layout.preferredWidth: 150
        Layout.preferredHeight: 40
        color: Theme.highlightColor
        radius: 15
        Label {
            anchors.centerIn: parent
            text: root.currentValue
            color: "white"
            font: Theme.defaultFont
        }
    }

    // 3. 设定值输入
    Rectangle {
        Layout.preferredWidth: 150
        Layout.preferredHeight: 40
        color: "transparent"
        border.color: Theme.highlightColor
        border.width: 2
        radius: 15
        TextInput {
            id: inputField
            anchors.fill: parent
            anchors.margins: 5
            verticalAlignment: Text.AlignVCenter
            horizontalAlignment: Text.AlignHCenter
            color: Theme.textColor
            font: Theme.defaultFont
            selectByMouse: true
            validator: root.inputValidator
            onActiveFocusChanged: parent.border.color = activeFocus ? Theme.orange : Theme.highlightColor
        }
    }

    // 4. 单位
    Label {
        text: root.unitText
        color: Theme.textColor
        font: Theme.defaultFont
        Layout.alignment: Qt.AlignVCenter
        Layout.preferredWidth: 40
    }

    // 【新增】右侧弹性占位符
    Item { Layout.fillWidth: true }
}
