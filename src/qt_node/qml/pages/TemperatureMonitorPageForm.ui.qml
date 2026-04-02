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
        height: 550
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

    // 右边中间：全局控制栏
    Rectangle {
        id: globalControlBar
        anchors.top: circuitPanle.bottom
        anchors.left: regulatorPanle.right
        anchors.right: parent.right
        anchors.topMargin: Theme.subSpacing
        anchors.leftMargin: Theme.subSpacing

        height: 60
        color: "transparent"
        border.color: Theme.highlightColor
        border.width: 2
        radius: 8

        RowLayout {
            anchors.fill: parent
            anchors.margins: 10
            spacing: 30

            Label { text: "控制模式:"; color: Theme.orange; font: Theme.defaultFont }

            // 替换为 ToggleActionButton（自带指示灯效果）
            ToggleActionButton {
                id: btnManualMode
                Layout.preferredWidth: 120
                Layout.preferredHeight: 40
                labelText: "手 动"
                colorWhenOn: "lime"
                indicatorOn: rosProxy.qmlSystemSettings && !rosProxy.qmlSystemSettings.auto_on
            }

            ToggleActionButton {
                id: btnAutoMode
                Layout.preferredWidth: 120
                Layout.preferredHeight: 40
                labelText: "自 动"
                colorWhenOn: "lime"
                indicatorOn: rosProxy.qmlSystemSettings && rosProxy.qmlSystemSettings.auto_on
            }

            Item { Layout.preferredWidth: 40 } // Spacer

            Label { text: "当前运行状态:"; color: Theme.orange; font: Theme.defaultFont }

            RowLayout {
                spacing: 15
                Rectangle {
                    width: 16; height: 16; radius: 8
                    color: rosProxy.systemStatus.circuit_work_status === 1 ? "lime" : "#333333"
                }
                Label { text: "回路 1 运行"; color: Theme.textColor; font: Theme.defaultFont }

                Rectangle {
                    width: 16; height: 16; radius: 8
                    color: rosProxy.systemStatus.circuit_work_status === 2 ? "lime" : "#333333"
                }
                Label { text: "回路 2 运行"; color: Theme.textColor; font: Theme.defaultFont }

                Rectangle {
                    width: 16; height: 16; radius: 8
                    color: rosProxy.systemStatus.circuit_work_status === 0 ? "orange" : "#333333"
                }
                Label { text: "全系统待机"; color: Theme.textColor; font: Theme.defaultFont }
            }

            Item { Layout.fillWidth: true } // 弹性占位
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
            legend.labelColor: "white"
            legend.font: Theme.smallLabelFont

            LineSeries {
                id: tempSeries
                name: "选中通道温度"
                axisX: axisX
                axisY: axisYTemp
                color: Theme.orange
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
                labelsColor: "white"
                gridLineColor: Theme.gridLineColor
                labelsFont: Theme.smallLabelFont
            }

            ValueAxis {
                id: axisYTemp
                min: 0
                max: 120
                tickCount: 7
                titleText: '<font color="' + Theme.orange + '">温度 (°C)</font>'
                titleFont: Theme.smallLabelFont
                labelsColor: "white"
                gridVisible: false
            }

            ValueAxis {
                id: axisYCurrent
                min: 0
                max: 3600
                tickCount: 7
                titleText: '<font color="' + Theme.titleColor + '">电流 (A)</font>'
                titleFont: Theme.smallLabelFont
                labelsColor: "white"
                gridLineColor: Theme.gridLineColor
            }
        }
    }
}
