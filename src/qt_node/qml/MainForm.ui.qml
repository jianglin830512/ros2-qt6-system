// 文件: qml/MainForm.ui.qml
import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import "./components"
import qt.theme 1.0

Rectangle {
    id: root
    color: Theme.backgroundColor

    // ==========================================
    // 暴露给 Main.qml 的接口 (Aliases)
    // ==========================================

    // 核心组件接口
    property alias stackLayout: contentLayout       // 用于页面切换
    property alias exitButton: exitButton           // 退出按钮
    property alias navModel: navRepeater.model      // 左侧导航栏的数据模型
    property alias dragArea: headerDragArea         // 暴露给外部处理拖拽逻辑

    // 状态与文本接口
    property alias timeLabel: timeLabel
    property alias statusLabel: statusLabel
    property alias remoteText: remoteText

    // 指示灯颜色接口
    property alias remoteIndicator: remoteIndicator
    property alias estopIndicator: estopIndicator

    // ==========================================
    // 1. 顶部 Header 区域
    // ==========================================
    Rectangle {
        id: header
        height: 90
        color: "transparent"
        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
            margins: Theme.mainSpacing
        }

        // 提供给 Main.qml 拖拽窗口用的 MouseArea
        MouseArea {
            id: headerDragArea
            anchors.fill: parent
        }

        // 左侧：时间与状态
        ColumnLayout {
            anchors {
                left: parent.left
                verticalCenter: parent.verticalCenter
                leftMargin: 10
            }
            spacing: 5
            Label { id: timeLabel; text: "1970-01-01 00:00:00"; color: Theme.textColor; font: Theme.defaultFont }
            Label { id: statusLabel; text: "系统状态: 初始化..."; color: Theme.textColor; font: Theme.defaultFont }
        }

        // 居中：系统大标题
        ColumnLayout {
            anchors.centerIn: parent
            spacing: 5
            Label {
                text: "电缆热循环试验测控系统"
                color: Theme.titleColor
                font: Theme.titleFont
                Layout.alignment: Qt.AlignHCenter
            }
            Label {
                text: "电力工业电气设备质量检验测试中心"
                color: Theme.textColor
                font: Theme.subTitleFont
                Layout.alignment: Qt.AlignHCenter
            }
        }

        // 右侧：指示灯组 (本地/远方、急停)
        RowLayout {
            anchors {
                right: parent.right
                verticalCenter: parent.verticalCenter
                rightMargin: 10
            }
            spacing: 15

            Rectangle {
                id: remoteIndicator
                width: 60; height: 60; radius: 4
                Text {
                    id: remoteText
                    anchors.centerIn: parent
                    font: Theme.indicatorFont
                    color: Theme.indicatorTextColor
                }
            }

            Rectangle {
                id: estopIndicator
                width: 60; height: 60; radius: 4
                Text {
                    anchors.centerIn: parent
                    text: "急停"
                    font: Theme.indicatorFont
                    color: Theme.indicatorTextColor
                }
            }
        }
    }

    // ==========================================
    // 2. 左侧导航菜单区
    // ==========================================
    ColumnLayout {
        id: menuLayout
        width: 135
        spacing: Theme.subSpacing
        anchors {
            top: header.bottom
            left: parent.left
            bottom: parent.bottom
            margins: Theme.mainSpacing
        }

        Repeater {
            id: navRepeater
            delegate: StyledButton {
                text: modelData
                Layout.fillWidth: true
                isSelected: contentLayout.currentIndex === index
                onClicked: contentLayout.currentIndex = index
            }
        }

        // 弹性占位，将退出按钮推至底部
        Item { Layout.fillHeight: true }

        StyledButton {
            id: exitButton
            text: "退出系统"
            Layout.fillWidth: true
        }
    }

    // ==========================================
    // 3. 右侧页面内容区
    // ==========================================
    StackLayout {
        id: contentLayout
        currentIndex: 0
        anchors {
            top: header.bottom
            left: menuLayout.right
            right: parent.right
            bottom: parent.bottom
            margins: Theme.mainSpacing
        }
    }
}
