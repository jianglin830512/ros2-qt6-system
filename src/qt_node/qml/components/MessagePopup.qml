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

    // 【核心修复 1】：固定一个较宽的宽度。
    // 去掉高度 height 属性，完全交给 Dialog 自动排版撑开，彻底杜绝重叠！
    width: 500

    modal: true
    title: isError ? "错误" : "提示"

    // 背景样式
    background: Rectangle {
        color: Theme.popupBackgroundColor
        border.color: root.isError ? Theme.errorColor : Theme.highlightColor
        border.width: 2
        radius: 10
    }

    // 标题栏
    header: Label {
        text: root.title
        color: root.isError ? Theme.errorColor : Theme.textColor
        font: Theme.titleFont
        padding: 15
        horizontalAlignment: Text.AlignHCenter
        background: Rectangle { color: "transparent" }
    }

    // 【核心修复 2】：简化内容区域，直接使用 Label。
    // 依靠 wrapMode 自动换行，依靠 padding 撑开上下空间，不再需要 Layout 计算。
    contentItem: Label {
        text: root.message
        color: Theme.textColor
        font: Theme.labelFont
        wrapMode: Text.Wrap
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter

        // 使用内边距提供“呼吸感”，避免文字贴着标题或按钮
        topPadding: 20
        bottomPadding: 30
        leftPadding: 20
        rightPadding: 20
    }

    // 底部按钮
    footer: DialogButtonBox {
        alignment: Qt.AlignHCenter
        background: Rectangle { color: "transparent" }
        padding: 10
        bottomPadding: 20 // 底部边缘留一点白

        Button {
            text: root.buttonText
            implicitWidth: 120
            implicitHeight: 40

            background: Rectangle {
                color: parent.down ? Qt.darker(Theme.highlightColor) : Theme.highlightColor
                radius: 5
                border.color: root.isError ? Theme.errorColor : Theme.highlightColor
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

    // 打开时的动画效果
    enter: Transition {
        NumberAnimation { property: "opacity"; from: 0.0; to: 1.0; duration: 100 }
        NumberAnimation { property: "scale"; from: 0.9; to: 1.0; duration: 100 }
    }
}
