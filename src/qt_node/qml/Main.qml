// 文件: qml/Main.qml
import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import "./components"
import "./pages"
import qt.theme 1.0

ApplicationWindow {
    id: window
    visible: true
    minimumWidth: 1920
    minimumHeight: 1080
    title: "电缆热循环试验测控系统"

    // 1. 设置启动时默认最大化
    visibility: Window.Maximized

    // 2. 自定义系统窗口标志 (取消关闭按钮)
    // - Qt.CustomizeWindowHint: 允许自定义标题栏按钮
    // - Qt.WindowTitleHint: 显示标题栏
    // - Qt.WindowSystemMenuHint: 显示系统菜单（必须包含以保证在Windows上正常渲染图标）
    // - Qt.WindowMinimizeButtonHint: 保留最小化按钮
    // - Qt.WindowMaximizeButtonHint: 保留最大化按钮
    // 注意：这里故意不包含 Qt.WindowCloseButtonHint，以此来隐藏/禁用关闭按钮
    flags: Qt.Window | Qt.CustomizeWindowHint | Qt.WindowTitleHint | Qt.WindowSystemMenuHint | Qt.WindowMinimizeButtonHint | Qt.WindowMaximizeButtonHint

    // ==========================================
    // 全局状态与 ROS 数据源
    // ==========================================
    readonly property bool isRemote: rosProxy.systemStatus.is_remote
    readonly property bool isEStop: rosProxy.systemStatus.emergency_stop_on
    readonly property bool mainOverCurrent: rosProxy.regulatorStatus1.over_current_on
    readonly property bool auxOverCurrent: rosProxy.regulatorStatus2.over_current_on
    readonly property bool anyOverCurrent: mainOverCurrent || auxOverCurrent

    // ==========================================
    // 核心 UI 实例化
    // ==========================================
    MainForm {
        id: ui
        anchors.fill: parent

        // 1. 设置侧边栏导航模型
        navModel: ["温度监控", "系统设置", "回路1设置", "回路2设置", "试验数据", "历史数据"]

        // 2. 指示灯状态绑定
        remoteIndicator.color: isRemote ? Theme.remoteGreen : Theme.localRed
        remoteText.text: isRemote ? "远方" : "本地"
        estopIndicator.color: isEStop ? (estopFlashTimer.flashState ? Theme.estopActiveRed : "black") : Theme.estopGray

        // 3. 窗口拖拽逻辑 (对接 MainForm 中暴露的 dragArea)
        property point dragPos: Qt.point(0, 0)
        dragArea.onPressed: (mouse) => { dragPos = Qt.point(mouse.x, mouse.y) }
        dragArea.onPositionChanged: (mouse) => {
            window.x += mouse.x - dragPos.x
            window.y += mouse.y - dragPos.y
        }

        // 4. 按钮交互事件
        exitButton.onClicked: exitDialog.open()

        // 5. 注入各个具体业务页面
        stackLayout.children: [
            TemperatureMonitorPage {},
            SystemSettingsPage {},
            CircuitSettingsPage {
                circuitId: 1
                settingsData: rosProxy.qmlCircuitSettings1
                statusData: rosProxy.circuitStatus1
            },
            CircuitSettingsPage {
                circuitId: 2
                settingsData: rosProxy.qmlCircuitSettings2
                statusData: rosProxy.circuitStatus2
            },
            TestDataPage {},
            HistoryPage {}
        ]
    }

    // ==========================================
    // 业务逻辑组件 (定时器与连接)
    // ==========================================

    // 时钟定时器
    Timer {
        interval: 1000
        running: true
        repeat: true
        onTriggered: {
            ui.timeLabel.text = Qt.formatDateTime(new Date(), "yyyy-MM-dd hh:mm:ss")
        }
    }

    // 急停指示灯闪烁定时器
    Timer {
        id: estopFlashTimer
        interval: 500
        running: window.isEStop
        repeat: true
        property bool flashState: false
        onTriggered: flashState = !flashState
        onRunningChanged: if (!running) flashState = false
    }

    // ROS 状态更新响应
    Connections {
        target: rosProxy
        function onQmlCircuitSettings1Changed() { updateMainStatus() }
        function onQmlCircuitSettings2Changed() { updateMainStatus() }
    }

    function updateMainStatus() {
        let isRunning = rosProxy.qmlCircuitSettings1.test_loop.enabled ||
                        rosProxy.qmlCircuitSettings2.test_loop.enabled;
        ui.statusLabel.text = "系统状态: " + (isRunning ? "运行中" : "待机")
    }

    // 过流报警弹窗触发逻辑
    onAnyOverCurrentChanged: {
        if (anyOverCurrent) {
            alarmPopup.open()
        }
    }

    // 拦截系统原生关闭事件
    onClosing: function(closeEvent) {
        closeEvent.accepted = false
        exitDialog.open()
    }

    // ==========================================
    // 弹窗与对话框定义
    // ==========================================

    // 退出确认对话框
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
            font: Theme.defaultFont
        }

        footer: DialogButtonBox {
            Button { text: "是"; onClicked: exitDialog.accept() }
            Button { text: "否"; onClicked: exitDialog.reject() }
        }

        onAccepted: {
            console.log("Shutting down via ROS Proxy...")
            rosProxy.initiateShutdown()
        }
    }

    // 严重告警弹窗
    Dialog {
        id: alarmPopup
        title: "系统严重告警"
        modal: true
        anchors.centerIn: parent
        width: 400
        height: 250
        closePolicy: Popup.NoAutoClose // 必须手动点击取消

        background: Rectangle {
            color: "#222222"
            border.color: "red"
            border.width: 3
            radius: 10
        }

        header: Label {
            text: "！ 调压器过流告警"
            color: "red"
            font: Theme.largeLabelFont
            horizontalAlignment: Text.AlignHCenter
            padding: 10
        }

        contentItem: ColumnLayout {
            spacing: 15
            Item { Layout.fillHeight: true }

            Label {
                text: "⚠️ 主调压器发生过流保护"
                color: "white"
                font: Theme.defaultFont
                visible: mainOverCurrent
                Layout.alignment: Qt.AlignHCenter
            }
            Label {
                text: "⚠️ 辅调压器发生过流保护"
                color: "white"
                font: Theme.defaultFont
                visible: auxOverCurrent
                Layout.alignment: Qt.AlignHCenter
            }

            Item { Layout.fillHeight: true }
        }

        footer: DialogButtonBox {
            alignment: Qt.AlignHCenter
            background: Rectangle { color: "transparent" }
            Button {
                text: "取消报警"
                contentItem: Text {
                    text: parent.text
                    color: "white"
                    font: Theme.buttonFont
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }
                background: Rectangle {
                    color: parent.pressed ? "#661111" : "#AA2222"
                    radius: 5
                }
                onClicked: {
                    rosProxy.sendClearAlarm()
                    alarmPopup.close()
                }
            }
        }
    }
}
