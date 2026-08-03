import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCharts
import qt.theme 1.0
import "../components"

Rectangle{
    id: pageRoot
    Layout.fillWidth: true
    Layout.fillHeight: true
    color: Theme.controlBgColor

    // --- Aliases for logic file ---
    property alias mainRegulator: mainRegulator
    property alias auxRegulator: auxRegulator
    property alias circuit1: circuit1
    property alias circuit2: circuit2
    property alias btnManualMode: btnManualMode
    property alias btnAutoMode: btnAutoMode

    // --- 图表相关别名 ---
    property alias tempSeries: tempSeries
    property alias currentSeries: currentSeries
    property alias axisX: axisX
    property alias axisYTemp: axisYTemp
    property alias axisYCurrent: axisYCurrent

    // --- 新增控制控件别名 ---
    property alias loopSelector: loopSelector
    property alias channelSelector: channelSelector
    property alias timeRangeSelector: timeRangeSelector

    // --- 左侧: 调压器 ---
    ColumnLayout {
        id: regulatorPanle
        anchors{
            left: parent.left
            top: parent.top
            bottom: parent.bottom
        }
        width: 280
        spacing: Theme.subSpacing
        RegulatorControl {
            id: mainRegulator
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
        RegulatorControl {
            id: auxRegulator
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }

    // 右上方: 回路状态面板
    RowLayout {
        id: circuitPanle
        anchors{
            left: regulatorPanle.right
            top: parent.top
            right: parent.right
            leftMargin: Theme.subSpacing
        }
        height: 720
        spacing: Theme.subSpacing

        CircuitStatus {
            id: circuit1
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
        CircuitStatus {
            id: circuit2
            Layout.fillWidth: true
            Layout.fillHeight: true
        }
    }

    // ==============================================
    // 右边中间：全局控制栏 (拆分为左右两个独立区块)
    // ==============================================
    RowLayout {
        id: globalControlBar
        anchors.top: circuitPanle.bottom
        anchors.left: regulatorPanle.right
        anchors.right: parent.right
        anchors.topMargin: Theme.subSpacing
        anchors.leftMargin: Theme.subSpacing

        height: 60
        spacing: Theme.subSpacing

        // --- 1. 左侧：手动/自动模式控制 ---
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "transparent"
            border.color: Theme.highlightColor
            border.width: 2
            radius: 8

            RowLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 30

                Item { Layout.fillWidth: true } // 左侧弹簧占位

                ToggleActionButton {
                    id: btnManualMode
                    Layout.preferredWidth: 140
                    Layout.preferredHeight: 45
                    labelText: "手 动"
                    colorWhenOn: "lime"
                    indicatorOn: rosProxy.qmlSystemSettings && !rosProxy.qmlSystemSettings.auto_on
                }

                ToggleActionButton {
                    id: btnAutoMode
                    Layout.preferredWidth: 140
                    Layout.preferredHeight: 45
                    labelText: "自 动"
                    colorWhenOn: "lime"
                    indicatorOn: rosProxy.qmlSystemSettings && rosProxy.qmlSystemSettings.auto_on
                }

                Item { Layout.fillWidth: true } // 右侧弹簧占位
            }
        }

        // --- 2. 中间：运行状态指示 ---
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "transparent"
            border.color: Theme.highlightColor
            border.width: 2
            radius: 8

            RowLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 8

                Item { Layout.fillWidth: true } // 左侧弹簧占位

                // [回路 1]
                Label { text: "回路 1 运行:"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignVCenter }
                Rectangle {
                    width: 16; height: 16; radius: 8; Layout.alignment: Qt.AlignVCenter
                    color: rosProxy.systemStatus.circuit_work_status === 1 ? Theme.statusOkColor : Theme.indicatorOffColor
                    border.color: Theme.indicatorBorderColor; border.width: 1
                }

                // 分割线
                Item { Layout.preferredWidth: 10 }
                Rectangle { width: 2; Layout.preferredHeight: 24; color: Theme.buttonBorderColor; Layout.alignment: Qt.AlignVCenter }
                Item { Layout.preferredWidth: 10 }

                // [回路 2]
                Label { text: "回路 2 运行:"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignVCenter }
                Rectangle {
                    width: 16; height: 16; radius: 8; Layout.alignment: Qt.AlignVCenter
                    // 根据要求回路2同样采取（灰/绿）配色
                    color: rosProxy.systemStatus.circuit_work_status === 2 ? Theme.statusOkColor : Theme.indicatorOffColor
                    border.color: Theme.indicatorBorderColor; border.width: 1
                }

                // 分割线
                Item { Layout.preferredWidth: 10 }
                Rectangle { width: 2; Layout.preferredHeight: 24; color: Theme.buttonBorderColor; Layout.alignment: Qt.AlignVCenter }
                Item { Layout.preferredWidth: 10 }

                // [系统待机]
                Label { text: "全系统待机:"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignVCenter }
                Rectangle {
                    width: 16; height: 16; radius: 8; Layout.alignment: Qt.AlignVCenter
                    // 根据要求待机采取（灰/金黄）配色
                    color: rosProxy.systemStatus.circuit_work_status === 0 ? Theme.statusStandbyColor : Theme.indicatorOffColor
                    border.color: Theme.indicatorBorderColor; border.width: 1
                }

                Item { Layout.fillWidth: true } // 右侧弹簧占位
            }
        }

        // --- 3. 右侧：设备连接状态指示 (新增面板) ---
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: "transparent"
            border.color: Theme.highlightColor
            border.width: 2
            radius: 8

            RowLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 8

                Item { Layout.fillWidth: true } // 左侧弹簧占位

                // [PLC连接]
                Label { text: "PLC 连接:"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignVCenter }
                Rectangle {
                    width: 16; height: 16; radius: 8; Layout.alignment: Qt.AlignVCenter
                    // 采取（红/绿）配色，先判定全局硬件连接是否正常，否则显示红
                    color: {
                        if (!rosProxy.systemStatus.hardware_connected) return Theme.errorColor;
                        return rosProxy.systemStatus.plc_connected ? Theme.statusOkColor : Theme.errorColor;
                    }
                    border.color: Theme.indicatorBorderColor; border.width: 1
                }

                // 分割线
                Item { Layout.preferredWidth: 10 }
                Rectangle { width: 2; Layout.preferredHeight: 24; color: Theme.buttonBorderColor; Layout.alignment: Qt.AlignVCenter }
                Item { Layout.preferredWidth: 10 }

                // [测温设备]
                Label { text: "测温设备:"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignVCenter }
                Rectangle {
                    width: 16; height: 16; radius: 8; Layout.alignment: Qt.AlignVCenter
                    color: {
                        if (!rosProxy.systemStatus.hardware_connected) return Theme.errorColor;
                        return rosProxy.systemStatus.temp_monitor_connected ? Theme.statusOkColor : Theme.errorColor;
                    }
                    border.color: Theme.indicatorBorderColor; border.width: 1
                }

                Item { Layout.fillWidth: true } // 右侧弹簧占位
            }
        }
    }

    // 右下方: 图表区域
    Rectangle {
        id: chartPanle
        anchors{
            left: regulatorPanle.right
            top: globalControlBar.bottom
            right: parent.right
            bottom: parent.bottom
            leftMargin: Theme.subSpacing
            topMargin: Theme.subSpacing
        }
        color: Theme.controlBgColor
        border.color: Theme.highlightColor
        border.width: 3

        // --- 顶部控制栏 ---
        RowLayout {
            id: controlBar
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.margins: 10
            height: 40
            spacing: 15
            z: 10

            Label { text: "数据源:"; color: Theme.textColor; font: Theme.defaultFont }
            ComboBox {
                id: loopSelector
                Layout.preferredWidth: 180
                model: ["回路1-试验回路", "回路1-模拟回路", "回路2-试验回路", "回路2-模拟回路"]
                font: Theme.smallLabelFont
            }

            Label { text: "温度通道:"; color: Theme.textColor; font: Theme.defaultFont }
            ComboBox {
                id: channelSelector
                Layout.preferredWidth: 120
                font: Theme.smallLabelFont
            }

            Label { text: "时间范围:"; color: Theme.textColor; font: Theme.defaultFont }
            ComboBox {
                id: timeRangeSelector
                Layout.preferredWidth: 120
                textRole: "text"
                valueRole: "value"
                model: [
                    { text: "10分钟", value: 10 },
                    { text: "30分钟", value: 30 },
                    { text: "1小时", value: 60 }
                ]
                font: Theme.smallLabelFont
                currentIndex: 0
            }

            Item { Layout.fillWidth: true }
        }

        // --- 图表视图 ---
        ChartView {
            id: chartView
            anchors.top: controlBar.bottom
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 5

            title: ""
            antialiasing: true
            backgroundColor: Theme.chartBgColor
            legend.visible: true
            legend.alignment: Qt.AlignTop
            legend.labelColor: Theme.textColor // 让图例文字颜色自适应主题
            legend.font: Theme.smallLabelFont

            LineSeries {
                id: tempSeries
                name: "选中通道温度"
                axisX: axisX
                axisY: axisYTemp
                color: Theme.statusHeatColor
                width: 2
            }
            LineSeries {
                id: currentSeries
                name: "回路电流"
                axisX: axisX
                axisYRight: axisYCurrent
                color: Theme.titleColor
                width: 2
            }

            DateTimeAxis {
                id: axisX
                format: "MM-dd hh:mm"
                tickCount: 5
                labelsColor: Theme.axisLabelColor
                gridLineColor: Theme.gridLineColor
                labelsFont: Theme.smallLabelFont
            }

            ValueAxis {
                id: axisYTemp
                min: -30
                max: 150
                tickCount: 7
                titleText: '<font color="' + Theme.statusHeatColor + '">温度 (°C)</font>'
                titleFont: Theme.smallLabelFont
                labelsColor: Theme.axisLabelColor
                gridVisible: false
            }

            ValueAxis {
                id: axisYCurrent
                min: 0
                max: 480
                tickCount: 7
                titleText: '<font color="' + Theme.titleColor + '">电流 (A)</font>'
                titleFont: Theme.smallLabelFont
                labelsColor: Theme.axisLabelColor
                gridLineColor: Theme.gridLineColor
            }
        }
    }
}
