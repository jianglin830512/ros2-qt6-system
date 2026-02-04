import QtQuick
import QtQuick.Controls
import qt.theme 1.0

Button {
    // 'root' 这个 id 引用的是 Button 本身。
    // 在 Main.qml 中使用的 StyledButton 实例会具有一个 isSelected 属性，
    // 我们在这里通过 root.isSelected 来访问它。
    id: root
    // implicitWidth: 200
    implicitHeight: 60

    // 使用一个基础的 Item 作为背景容器，以便我们在内部进行分层
    background: Item {
        anchors.fill: parent

        // --- 第1层：常规/悬停状态的背景和边框 ---
        // 这一层始终可见，但它的颜色会变化
        Rectangle {
            anchors.fill: parent
            radius: 8

            // 边框只在未选中时显示
            border.width: root.isSelected ? 0 : 2
            border.color: Theme.buttonBorderColor

            // 颜色逻辑：
            // 1. 如果鼠标悬停 并且 按钮未被选中，则显示高亮色
            // 2. 否则，显示默认的透明背景色
            color: root.hovered && !root.isSelected ? Theme.buttonHoverColor : Theme.buttonBackgroundColor
        }

        // --- 第2层：选中状态的渐变背景 ---
        // 这一层只有在按钮被选中时才可见
        Rectangle {
            anchors.fill: parent
            radius: 8
            visible: root.isSelected // 这是控制显示/隐藏的关键

            // Qt 6+ 的现代渐变语法
            // 直接在 Rectangle 上设置 gradient 属性
            gradient: Gradient {
                // 设置渐变方向为垂直
                orientation: Gradient.Vertical

                // 定义渐变的起始和结束颜色点
                GradientStop { position: 0.0; color: Theme.buttonSelectedGradientStart }
                GradientStop { position: 1.0; color: Theme.buttonSelectedGradientEnd }
            }
        }
    }

    contentItem: Text {
        text: root.text
        font: Theme.buttonFont

        // 文本颜色根据是否选中来变化
        color: root.isSelected ? Theme.buttonSelectedTextColor : Theme.textColor

        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
    }
}
