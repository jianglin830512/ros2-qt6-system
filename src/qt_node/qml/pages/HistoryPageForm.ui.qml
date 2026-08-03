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
                            // model 在逻辑层设置，现在是一个 ListModel
                            delegate: CheckBox {
                                text: model.label !== undefined ? model.label : ""
                                checked: false

                                // 自定义选框，增加中间颜色块来指示图例颜色
                                indicator: Rectangle {
                                    implicitWidth: 18
                                    implicitHeight: 18
                                    x: parent.leftPadding
                                    y: parent.height / 2 - height / 2
                                    radius: 3
                                    border.color: parent.checked ? Theme.highlightColor : Theme.checkboxUncheckedColor
                                    color: "transparent"

                                    // 内部实心块显示对应的曲线颜色
                                    Rectangle {
                                        anchors.centerIn: parent
                                        width: 12
                                        height: 12
                                        radius: 2
                                        visible: parent.checked && model.lineColor !== undefined && model.lineColor !== ""
                                        color: model.lineColor !== undefined && model.lineColor !== "" ? model.lineColor : "transparent"
                                    }
                                }

                                contentItem: Text {
                                    text: parent.text
                                    font: Theme.smallLabelFont
                                    // 被选中并生成了图表 lineColor 时，采用 lineColor；否则用普通的选中高亮色
                                    color: parent.checked
                                           ? ((model.lineColor !== undefined && model.lineColor !== "") ? model.lineColor : Theme.checkboxCheckedColor)
                                           : Theme.checkboxUncheckedColor
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
                            width: 140; height: 36
                            color: "transparent"
                            border.color: Theme.highlightColor
                            border.width: 2
                            radius: 10 // 统一圆角
                            TextInput {
                                id: dateInput
                                anchors.fill: parent; anchors.margins: 5
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignHCenter
                                color: Theme.textColor // 深碳灰色
                                font: Theme.defaultFont
                                text: Qt.formatDateTime(new Date(), "yyyy-MM-dd")
                                validator: RegularExpressionValidator { regularExpression: /^[0-9\-\/\.]+$/ }
                                // 添加焦点变色逻辑（与回路设置一致）
                                onActiveFocusChanged: parent.border.color = activeFocus ? Theme.orange : Theme.highlightColor
                            }
                        }
                    }

                    // 时间
                    RowLayout {
                        Label { text: "时间:"; color: Theme.textColor; font: Theme.defaultFont }
                        Rectangle {
                            width: 80; height: 36
                            color: "transparent"
                            border.color: Theme.highlightColor
                            border.width: 2
                            radius: 10
                            TextInput {
                                id: timeInput
                                anchors.fill: parent; anchors.margins: 5
                                verticalAlignment: Text.AlignVCenter
                                horizontalAlignment: Text.AlignHCenter
                                color: Theme.textColor
                                font: Theme.defaultFont
                                text: Qt.formatDateTime(new Date(), "hh:mm")
                                validator: RegularExpressionValidator { regularExpression: /^[0-9:]+$/ }
                                onActiveFocusChanged: parent.border.color = activeFocus ? Theme.orange : Theme.highlightColor
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
                    }
                }
            }

            ChartView {
                id: chartView
                Layout.fillWidth: true
                Layout.fillHeight: true
                theme: ChartView.ChartThemeDark
                antialiasing: true
                backgroundColor: Theme.chartBgColor
                legend.alignment: Qt.AlignBottom
                legend.labelColor: "white"

                // --- 移除 axes: [ 和 ]，直接放置坐标轴 ---

                // X 轴：时间
                DateTimeAxis {
                    id: axisX
                    format: "MM-dd hh:mm"
                    tickCount: 8
                    labelsColor: "white"
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
