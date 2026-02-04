import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import "./components"
import qt.theme 1.0

Rectangle {
    id: root
    color: Theme.backgroundColor

    // --- 暴露给 Main.qml 的接口 ---
    property alias exitButton: exitButton
    readonly property alias stackLayout: contentLayout
    readonly property alias timeLabel: timeLabel
    readonly property alias statusLabel: statusLabel
    readonly property alias buttonContainer: buttonContainer

    // 1. 头部
    Rectangle {
        id: header
        height: 90
        color: "transparent"
        anchors { top: parent.top; left: parent.left; right: parent.right; margins: Theme.mainSpacing }
        ColumnLayout {
            anchors { left: parent.left; verticalCenter: parent.verticalCenter }
            Label { id: timeLabel; text: "2025-09-15 15:31:22"; color: Theme.textColor; font: Theme.defaultFont }
            Label { id: statusLabel; text: "系统状态: 正常"; color: Theme.textColor; font: Theme.defaultFont }
        }
        ColumnLayout {
            anchors.centerIn: parent; spacing: 5
            Label { text: "电缆热循环试验测控系统"; color: Theme.titleColor; font: Theme.titleFont; Layout.alignment: Qt.AlignHCenter }
            Label { text: "电力工业电气设备质量检验测试中心"; color: Theme.textColor; font: Theme.subTitleFont; Layout.alignment: Qt.AlignHCenter }
        }
    }

    // 2. 左侧菜单
    ColumnLayout {
        id: menuLayout
        width: 135
        anchors {
            top: header.bottom;
            left: parent.left;
            bottom: parent.bottom;
            margins: Theme.mainSpacing
        }
        spacing: Theme.subSpacing

        // 这个容器专门用来存放由 Repeater 动态创建的按钮
        ColumnLayout {
            id: buttonContainer
            Layout.fillWidth: true
            spacing: Theme.subSpacing
        }

        // 伸缩项，把退出按钮推到底部
        Item { Layout.fillHeight: true }

        StyledButton {
            id: exitButton
            Layout.fillWidth: true
            text: "退出系统"
        }
    }

    // 3. 右侧内容区
    StackLayout {
        id: contentLayout
        currentIndex: 0
        anchors {
            top: header.bottom;
            left: menuLayout.right;
            right: parent.right;
            bottom: parent.bottom;
            margins: Theme.mainSpacing
        }
    }
}
