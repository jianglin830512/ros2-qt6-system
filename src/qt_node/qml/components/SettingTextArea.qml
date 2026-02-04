import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

ColumnLayout {
    id: root
    property string title: "标题"
    property alias text: textArea.text

    // 支持反馈值/设定值分离模式
    property string currentValue: ""
    property alias settingValue: textArea.text

    // 【新增】允许外部控制输入框高度
    property int inputHeight: 80

    spacing: 5

    Label {
        text: root.title
        color: Theme.orange
        font: Theme.defaultFont
        Layout.alignment: Qt.AlignHCenter
    }

    // 容器
    Rectangle {
        Layout.fillWidth: true
        // 【修改】绑定到新属性
        Layout.preferredHeight: root.inputHeight
        color: "transparent"
        border.color: Theme.highlightColor
        border.width: 2
        radius: 10

        TextArea {
            id: textArea
            anchors.fill: parent
            anchors.margins: 10
            color: Theme.textColor
            font: Theme.defaultFont
            wrapMode: TextEdit.Wrap
            placeholderText: root.currentValue
            placeholderTextColor: Qt.rgba(1, 1, 1, 0.3)
            background: null
            // 垂直居中
            verticalAlignment: TextEdit.AlignVCenter
        }
    }
}
