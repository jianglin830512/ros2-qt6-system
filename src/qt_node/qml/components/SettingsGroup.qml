import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

Rectangle {
    id: root
    // 【关键】将默认内容指向内部的 ColumnLayout，这样子项会自动垂直排列
    default property alias content: contentLayout.data

    // 暴露按钮，以便外部通过别名访问 (如 group.applyButton)
    property alias applyButton: applyButton
    property alias restoreButton: restoreButton

    property string title: "标题"
    property alias spacing: contentLayout.spacing

    Layout.fillHeight: true
    Layout.fillWidth: true

    color: "transparent"
    border.color: Theme.titleColor
    border.width: 2
    radius: 10

    // === 1. 顶部固定区域：标题 ===
    Item {
        id: titleArea
        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
        }
        height: 90 // 稍微调小了一点，140可能太高了，你可以根据实际喜好改回 140

        Label {
            anchors.centerIn: parent
            text: root.title
            font: Theme.titleFont
            color: Theme.orange
        }
    }

    // === 2. 中间弹性区域：内容 ===
    Item {
        id: container
        anchors {
            top: titleArea.bottom
            left: parent.left
            right: parent.right
            bottom: applyButtonArea.top
        }

        // 【新增】布局管理器，让输入框自动居中且垂直排列
        ColumnLayout {
            id: contentLayout
            anchors{
                fill: parent
                margins: 20
            }
            spacing: 20
        }
    }

    // === 3. 底部固定区域：按钮 ===
    Item {
        id: applyButtonArea
        anchors {
            left: parent.left
            right: parent.right
            bottom: parent.bottom
            bottomMargin: 10
        }
        height: 80

        // 【修改】改为 RowLayout 以放置两个按钮
        RowLayout {
            anchors.centerIn: parent
            spacing: 20

            // 1. 还原按钮
            StyledButton {
                id: restoreButton
                text: "还原"
                // 样式微调：可以让它看起来像“次要操作”，比如文字颜色不同，或者保持一致
                // 这里暂时保持一致，通过文字区分
                implicitWidth: 120
                implicitHeight: 50
            }

            // 2. 应用按钮
            StyledButton {
                id: applyButton
                text: "应用"
                implicitWidth: 120 // 稍微改窄一点，让两个按钮排得下
                implicitHeight: 50
                // 简单的视觉区分：应用按钮被选中时稍微亮一点
                isSelected: applyButton.down
            }
        }
    }
}

