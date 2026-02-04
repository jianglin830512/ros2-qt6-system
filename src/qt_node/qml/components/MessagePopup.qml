import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

Dialog {
    id: root

    // --- Public API ---
    property string message: ""
    property bool isError: true // 默认为错误弹窗(红色)，设为false则为普通提示(主题色)
    property string buttonText: "确定"

    // 居中显示
    anchors.centerIn: Overlay.overlay
    width: 400
    height: 240
    modal: true
    title: isError ? "错误" : "提示"

    // 背景样式
    background: Rectangle {
        color: Theme.backgroundColor
        // 根据是否报错显示红边框或主题色边框
        border.color: root.isError ? "red" : Theme.highlightColor
        border.width: 2
        radius: 10
    }

    // 标题栏
    header: Label {
        text: root.title
        color: root.isError ? "red" : Theme.textColor
        font: Theme.titleFont
        padding: 15
        horizontalAlignment: Text.AlignHCenter
        background: Rectangle { color: "transparent" }
    }

    // 内容区域
    contentItem: ColumnLayout {
        spacing: 20

        // 稍微加点弹性空间让文字居中
        Item { Layout.fillHeight: true }

        Label {
            text: root.message
            color: Theme.textColor
            font: Theme.defaultFont
            Layout.fillWidth: true
            Layout.leftMargin: 20
            Layout.rightMargin: 20
            wrapMode: Text.Wrap
            horizontalAlignment: Text.AlignHCenter
        }

        Item { Layout.fillHeight: true }
    }

    // 底部按钮
    footer: DialogButtonBox {
        alignment: Qt.AlignHCenter
        background: Rectangle { color: "transparent" }
        padding: 10

        Button {
            text: root.buttonText

            // 复用 StyledButton 的一些样式逻辑
            background: Rectangle {
                color: parent.down ? Qt.darker(Theme.highlightColor) : Theme.highlightColor
                radius: 5
                border.color: root.isError ? "red" : Theme.highlightColor
                border.width: root.isError ? 1 : 0
            }
            contentItem: Text {
                text: parent.text
                color: "white"
                font: Theme.buttonFont
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }

            // 点击直接关闭自己
            onClicked: root.close()
        }
    }

    // 打开时的动画效果（可选）
    enter: Transition {
        NumberAnimation { property: "opacity"; from: 0.0; to: 1.0; duration: 100 }
        NumberAnimation { property: "scale"; from: 0.9; to: 1.0; duration: 100 }
    }
}
