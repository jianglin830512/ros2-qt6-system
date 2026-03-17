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

    // 1. 标签
    Label {
        text: root.labelText
        color: Theme.textColor
        font: Theme.defaultFont
        Layout.alignment: Qt.AlignVCenter
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

            // 边框高亮效果
            onActiveFocusChanged: parent.border.color = activeFocus ? Theme.orange : Theme.highlightColor

            // 【核心修复】：当输入完毕（失去焦点或按回车）时，主动强制限制在上下限范围内
            onEditingFinished: {
                if (validator !== null) {
                    var min = validator.bottom;
                    var max = validator.top;

                    // 确保是具有 bottom 和 top 属性的验证器 (排除正则验证器)
                    if (min !== undefined && max !== undefined) {
                        var val = parseFloat(text);
                        // 如果用户删光了输入框，或者数值小于最小值，强制设为最小值
                        if (isNaN(val) || val < min) {
                            text = min.toString();
                        }
                        // 如果数值大于最大值，强制设为最大值
                        else if (val > max) {
                            text = max.toString();
                        }
                    }
                }
            }
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

    Item { Layout.fillWidth: true }
}
