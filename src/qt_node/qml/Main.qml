// 文件: qml/Main.qml (最终版)
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "./components"
import "./pages"
import qt.theme 1.0

ApplicationWindow {
    id: window
    visible: true
    //width: 1920
    //height: 1080
    minimumWidth: 1920
    minimumHeight: 1080
    title: "电缆热循环试验测控系统"
    //flags: Qt.Window | Qt.FramelessWindowHint

    // 加载UI骨架
    MainForm {
        id: ui
        anchors.fill: parent

        // 1. 直接连接到暴露出来的退出按钮的 onClicked 信号
        exitButton.onClicked: {
            console.log("Exit button clicked, calling window.close()...")
            window.close()
        }

        // 2. 声明式地将所有页面定义为 StackLayout 的子项
        stackLayout.children: [
            TemperatureMonitorPage {},
            SystemSettingsPage {},

            // --- 回路1 ---
            CircuitSettingsPage {
                circuitId: 1
                settingsData: rosProxy.qmlCircuitSettings1
                statusData: rosProxy.circuitStatus1
            },

            // --- 回路2 ---
            CircuitSettingsPage {
                circuitId: 2
                settingsData: rosProxy.qmlCircuitSettings2
                statusData: rosProxy.circuitStatus2
            },

            TestDataPage {},
            HistoryPage {}
        ]
    }

    // 3. 将 Repeater 的内容动态地创建到 UI 提供的 buttonContainer 容器中
    Repeater {
        parent: ui.buttonContainer
        model: ["温度监控", "系统设置", "回路1设置", "回路2设置", "试验数据", "历史数据"]

        delegate: StyledButton {
            text: modelData
            Layout.fillWidth: true
            isSelected: ui.stackLayout.currentIndex === index

            onClicked: {
                ui.stackLayout.currentIndex = index
            }
        }
    }

    // --- 后台逻辑部分 ---
    Timer {
        interval: 1000
        running: true
        repeat: true
        onTriggered: {
            ui.timeLabel.text = Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
        }
    }

    Connections {
        target: rosProxy
        function onQmlCircuitSettings1Changed() {
            updateMainStatus()
        }
        function onQmlCircuitSettings2Changed() {
            updateMainStatus()
        }
    }

    MouseArea {
        anchors.fill: ui.header
        property var previousPosition
        onPressed: previousPosition = Qt.point(mouse.x, mouse.y)
        onPositionChanged: {
            var delta = Qt.point(mouse.x - previousPosition.x, mouse.y - previousPosition.y)
            window.setX(window.x + delta.x)
            window.setY(window.y + delta.y)
        }
    }

    // 对话框组件
    Dialog {
        id: exitDialog
        title: "确认退出"
        modal: true
        width: 300
        height: 150
        anchors.centerIn: parent
        Text {
            anchors.centerIn: parent
            text: "您确定要关闭应用程序吗？"
        }
        footer: DialogButtonBox {
            Button {
                text: "是"
                onClicked: {
                    exitDialog.accept()
                }
            }
            Button {
                text: "否"
                onClicked: {
                    exitDialog.reject()
                }
            }
        }
        onAccepted: {
            rosProxy.initiateShutdown()
        }
    }

    onClosing: (close) => {
                   // 取消默认的关闭行为
                   close.accepted = false;
                   // 然后弹出我们自定义的对话框
                   exitDialog.open();
               }

    function updateMainStatus() {
        // 只要有一个试验回路被启用，系统就是运行中
        var isRunning = rosProxy.qmlCircuitSettings1.test_loop.enabled || rosProxy.qmlCircuitSettings2.test_loop.enabled
        ui.statusLabel.text = "系统状态: " + (isRunning ? "运行中" : "待机")
    }
}
