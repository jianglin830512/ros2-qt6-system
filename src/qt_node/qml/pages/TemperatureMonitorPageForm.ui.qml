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

    // 右下方: 图表区域
    Rectangle {
        id: chartPanle
        anchors{
            left: regulatorPanle.right
            top: circuitPanle.bottom
            right: parent.right
            bottom: parent.bottom
            leftMargin: Theme.subSpacing
            topMargin: Theme.subSpacing
        }
        color: Theme.controlBgColor
        border.color: Theme.highlightColor
        border.width: 3

        // --- 顶部控制栏 (新增) ---
        RowLayout {
            id: controlBar
            anchors.top: parent.top
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.margins: 10
            height: 40
            spacing: 15
            z: 10 // 确保显示在图表之上

            // 1. 回路选择
            Label { text: "数据源:"; color: Theme.textColor; font: Theme.defaultFont }
            ComboBox {
                id: loopSelector
                Layout.preferredWidth: 180
                model: ["回路1-试验回路", "回路1-模拟回路", "回路2-试验回路", "回路2-模拟回路"]
                font: Theme.smallLabelFont
            }

            // 2. 通道选择
            Label { text: "温度通道:"; color: Theme.textColor; font: Theme.defaultFont }
            ComboBox {
                id: channelSelector
                Layout.preferredWidth: 120
                // model 由逻辑层动态设置
                font: Theme.smallLabelFont
            }

            // 3. 时间范围选择
            Label { text: "时间范围:"; color: Theme.textColor; font: Theme.defaultFont }
            ComboBox {
                id: timeRangeSelector
                Layout.preferredWidth: 120
                textRole: "text"
                valueRole: "value"
                model: [
                    { text: "2小时", value: 2 },
                    { text: "4小时", value: 4 },
                    { text: "8小时", value: 8 },
                    { text: "12小时", value: 12 },
                    { text: "24小时", value: 24 }
                ]
                font: Theme.smallLabelFont
                currentIndex: 0 // 默认2小时
            }

            Item { Layout.fillWidth: true } // 占位符
        }

        // --- 图表视图 ---
        ChartView {
            id: chartView
            anchors.top: controlBar.bottom // 在控制栏下方
            anchors.left: parent.left
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            anchors.bottomMargin: 5

            title: "" // 标题通过外部Label或Dropdown体现即可
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

            // --- 坐标轴定义 ---
            // 1. X轴改为 DateTimeAxis
            DateTimeAxis {
                id: axisX
                format: "MM-dd hh:mm" // 月-日 时:分
                tickCount: 5
                labelsColor: "white"
                gridLineColor: Theme.gridLineColor
                labelsFont: Theme.smallLabelFont
            }

            // 2. 左侧Y轴 (温度)
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

            // 3. 右侧Y轴 (电流)
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
