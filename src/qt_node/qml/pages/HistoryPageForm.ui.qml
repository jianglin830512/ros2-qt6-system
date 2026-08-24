import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCharts
import qt.theme 1.0
import "../components"

Item {
    id: root

    // Aliases
    property alias queryPanel: queryPanel
    property alias colRepeater: colRepeater
    property alias chartView: chartView
    property alias axisX: axisX
    property alias axisYTemp: axisYTemp
    property alias axisYVoltage: axisYVoltage
    property alias axisYCurrent: axisYCurrent

    // 布局
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        // === 上方：通用查询控制栏 ===
        QueryPanel {
            id: queryPanel
        }

        // === 下方：列选择与图表 ===
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 10

            // 1. 左下：列选择面板
            Rectangle {
                Layout.fillHeight: true
                Layout.preferredWidth: 280
                color: Theme.controlBgColor
                border.color: Theme.highlightColor
                border.width: 2
                radius: 5

                ColumnLayout {
                    anchors.fill: parent
                    anchors.margins: 10

                    Label {
                        text: "数据列选择 (最多5项)"
                        color: Theme.orange
                        font: Theme.defaultFont
                        Layout.alignment: Qt.AlignHCenter
                    }

                    // 滚动区域
                    ScrollView {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        clip: true
                        ScrollBar.vertical.policy: ScrollBar.AlwaysOn

                        Column {
                            width: parent.width
                            spacing: 2

                            Repeater {
                                id: colRepeater
                                delegate: CheckBox {
                                    id: colCheckBox
                                    text: typeof model !== "undefined" && model.label ? model.label : ""
                                    checked: false

                                    indicator: Rectangle {
                                        implicitWidth: 18
                                        implicitHeight: 18
                                        x: colCheckBox.leftPadding
                                        y: colCheckBox.height / 2 - height / 2
                                        radius: 3
                                        border.color: colCheckBox.checked ? Theme.highlightColor : Theme.checkboxUncheckedColor
                                        color: "transparent"

                                        Rectangle {
                                            anchors.centerIn: parent
                                            width: 12
                                            height: 12
                                            radius: 2
                                            visible: colCheckBox.checked ? (typeof model !== "undefined" && typeof model.lineColor === "string" && model.lineColor !== "") : false
                                            color: (typeof model !== "undefined" && typeof model.lineColor === "string" && model.lineColor !== "") ? model.lineColor : "transparent"
                                        }
                                    }

                                    contentItem: Text {
                                        text: colCheckBox.text
                                        font: Theme.smallLabelFont
                                        color: colCheckBox.checked
                                            ? ((typeof model !== "undefined" && typeof model.lineColor === "string" && model.lineColor !== "") ? model.lineColor : Theme.checkboxCheckedColor)
                                            : Theme.checkboxUncheckedColor
                                        verticalAlignment: Text.AlignVCenter
                                        leftPadding: colCheckBox.indicator.width + colCheckBox.spacing
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // 2. 右下：图表展示区域
            ChartView {
                id: chartView
                Layout.fillWidth: true
                Layout.fillHeight: true
                theme: ChartView.ChartThemeDark
                antialiasing: true
                backgroundColor: Theme.chartBgColor
                legend.alignment: Qt.AlignBottom
                legend.labelColor: "white"

                // X 轴：包含日期和时间的可见刻度
                DateTimeAxis {
                    id: axisX
                    format: "MM-dd hh:mm"
                    tickCount: 8
                    labelsColor: Theme.axisLabelColor
                    labelsFont: Theme.smallLabelFont
                    gridLineColor: Theme.gridLineColor
                }

                // 左侧 Y 轴 1：温度 (橙色)
                ValueAxis {
                    id: axisYTemp
                    labelsColor: Theme.statusHeatColor
                    titleText: "<font color='" + Theme.statusHeatColor + "'>温度 (°C)</font>"
                    gridLineColor: Theme.gridLineColor
                    visible: false
                }

                // 左侧 Y 轴 2：电压 (黄色)
                ValueAxis {
                    id: axisYVoltage
                    labelsColor: Theme.chartVoltageColor
                    titleText: "<font color='#E0E000'>电压 (V)</font>"
                    gridVisible: false
                    visible: false
                }

                // 右侧 Y 轴：电流 (青蓝色)
                ValueAxis {
                    id: axisYCurrent
                    labelsColor: Theme.titleColor
                    titleText: "<font color='" + Theme.titleColor + "'>电流 (A)</font>"
                    gridVisible: false
                    visible: false
                }
            }
        }
    }
}
