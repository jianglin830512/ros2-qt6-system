import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCharts
import qt.theme 1.0
import "../components"

Item {
    id: root

    // Aliases
    property alias dateInput: dateInput
    property alias timeInput: timeInput
    property alias spanCombo: spanCombo
    property alias queryBtn: queryBtn
    property alias colRepeater: colRepeater
    property alias chartView: chartView
    property alias axisX: axisX
    property alias axisYTemp: axisYTemp
    property alias axisYVoltage: axisYVoltage
    property alias axisYCurrent: axisYCurrent

    // 布局
    RowLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        // === 左侧：列选择面板 ===
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
                    text: "数据列选择 (最多10项)"
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
                            // model 在逻辑层设置
                            delegate: CheckBox {
                                text: modelData.label
                                checked: false
                                // 简单的样式覆盖
                                contentItem: Text {
                                    text: parent.text
                                    font: Theme.smallLabelFont
                                    color: parent.checked ? "white" : "#AAAAAA"
                                    verticalAlignment: Text.AlignVCenter
                                    leftPadding: parent.indicator.width + parent.spacing
                                }
                            }
                        }
                    }
                }
            }
        }

        // === 右侧：控制栏 + 图表 ===
        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 10

            // 1. 顶部查询条
            Rectangle {
                Layout.fillWidth: true
                Layout.preferredHeight: 60
                color: Theme.controlBgColor
                border.color: Theme.highlightColor
                border.width: 2
                radius: 5

                RowLayout {
                    anchors.fill: parent
                    anchors.margins: 10
                    spacing: 20

                    // 日期
                    RowLayout {
                        Label { text: "日期:"; color: Theme.textColor; font: Theme.defaultFont }
                        Rectangle {
                            width: 140; height: 36; color: Theme.highlightColor; radius: 5
                            TextInput {
                                id: dateInput
                                anchors.fill: parent; anchors.leftMargin: 5
                                verticalAlignment: Text.AlignVCenter
                                color: "white"; font: Theme.defaultFont
                                text: Qt.formatDateTime(new Date(), "yyyy-MM-dd")
                                inputMask: "9999-99-99" // 简单掩码
                            }
                        }
                    }

                    // 时间
                    RowLayout {
                        Label { text: "时间:"; color: Theme.textColor; font: Theme.defaultFont }
                        Rectangle {
                            width: 80; height: 36; color: Theme.highlightColor; radius: 5
                            TextInput {
                                id: timeInput
                                anchors.fill: parent; anchors.leftMargin: 5
                                verticalAlignment: Text.AlignVCenter
                                color: "white"; font: Theme.defaultFont
                                text: Qt.formatDateTime(new Date(), "hh:mm")
                                inputMask: "99:99"
                            }
                        }
                    }

                    // 跨度
                    RowLayout {
                        Label { text: "时长:"; color: Theme.textColor; font: Theme.defaultFont }
                        ComboBox {
                            id: spanCombo
                            width: 100
                            model: [2, 4, 8, 12, 24]
                            currentIndex: 0
                        }
                        Label { text: "小时"; color: Theme.textColor; font: Theme.defaultFont }
                    }

                    Item { Layout.fillWidth: true }

                    // 查询按钮
                    StyledButton {
                        id: queryBtn
                        text: "查询"
                        implicitWidth: 120
                        implicitHeight: 40
                        // 样式微调，使其显眼
                        background: Rectangle {
                            color: Theme.highlightColor
                            border.color: Theme.orange
                            border.width: 2
                            radius: 5
                        }
                    }
                }
            }

            // 2. 图表区域
            ChartView {
                id: chartView
                Layout.fillWidth: true
                Layout.fillHeight: true
                theme: ChartView.ChartThemeDark
                antialiasing: true
                backgroundColor: Theme.chartBgColor
                legend.alignment: Qt.AlignBottom
                legend.labelColor: "white"

                // --- 明确声明所有可用的坐标轴 ---
                axes: [
                    // X 轴：时间
                    DateTimeAxis {
                        id: axisX
                        format: "MM-dd hh:mm"
                        tickCount: 8
                        labelsColor: "white"
                        gridLineColor: Theme.gridLineColor
                    },

                    // 左侧 Y 轴 1：温度 (橙色)
                    ValueAxis {
                        id: axisYTemp
                        labelsColor: Theme.orange
                        // 【修改】删除 titleBrush，使用 HTML 标签包裹标题文本
                        titleText: "<font color='" + Theme.orange + "'>温度 (°C)</font>"
                        gridLineColor: Theme.gridLineColor
                        visible: false // 默认隐藏
                    },

                    // 左侧 Y 轴 2：电压 (黄色)
                    ValueAxis {
                        id: axisYVoltage
                        labelsColor: "#E0E000" // 明黄色
                        // 【修改】删除 titleBrush，使用 HTML 标签包裹标题文本
                        titleText: "<font color='#E0E000'>电压 (V)</font>"
                        gridVisible: false // 隐藏副轴网格线
                        visible: false
                    },

                    // 右侧 Y 轴：电流 (青蓝色)
                    ValueAxis {
                        id: axisYCurrent
                        labelsColor: Theme.titleColor
                        // 【修改】删除 titleBrush，使用 HTML 标签包裹标题文本
                        titleText: "<font color='" + Theme.titleColor + "'>电流 (A)</font>"
                        gridVisible: false
                        visible: false
                    }
                ]
            }
        }
    }
}
