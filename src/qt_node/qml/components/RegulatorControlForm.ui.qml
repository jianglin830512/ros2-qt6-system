import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtCharts
import qt.theme 1.0

Rectangle {
    id: root
    color: Theme.controlBgColor
    border.color: Theme.highlightColor
    border.width: 3

    // --- Aliases for logic file ---
    property alias titleLabel: titleLabel
    property alias overCurrentLabel: overCurrentLabel
    property alias voltageLabel: voltageLabel
    property alias currentLabel: currentLabel
    property alias upArrow: upArrow
    property alias downArrow: downArrow
    property alias closeBreakerButton: closeBreakerButton
    property alias openBreakerButton: openBreakerButton
    property alias voltageUpButton: voltageUpButton
    property alias voltageDownButton: voltageDownButton

    // 电压升降指示的宽度
    property int voltageIndicatorWidth: 30

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10
        Rectangle{
            color: Theme.controlBgColor
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 20
            RowLayout {
                anchors.fill: parent
                Item { Layout.fillWidth: true } // Spacer
                Item {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Label {
                        id: titleLabel
                        anchors.centerIn: parent
                        text: "调压器"
                        font: Theme.labelFont
                        color: Theme.orange
                    }
                }
                Item {
                    Layout.fillWidth: true
                    Layout.fillHeight: true

                    Label {
                        id: overCurrentLabel
                        text: "过流"
                        color: "red"
                        visible: false
                        font: Theme.defaultFont
                    }
                }
            }
        }

        Rectangle{
            color: Theme.controlBgColor
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 60
            RowLayout {
                anchors.fill: parent
                spacing: Theme.subSpacing
                ToggleActionButton {
                    id: closeBreakerButton
                    labelText: "合闸"
                    colorWhenOn: "red"
                }
                ToggleActionButton {
                    id: openBreakerButton
                    labelText: "分闸"
                    colorWhenOn: "lime"
                }
            }
        }

        // --- Placeholder for Chart ---
        Rectangle {
            Layout.fillWidth: true
            Layout.minimumHeight: 200
            Layout.fillHeight: true
            color: "white"
            border.color: Theme.highlightColor
            ChartView {
                id: chartView
                anchors.fill: parent
                anchors.bottomMargin: 15
                antialiasing: true
                legend.visible: false // 隐藏图例
                backgroundColor: Theme.controlBgColor

                axes: [

                    // 使用横向的条形图类别来并列显示两个柱状图
                    BarCategoryAxis {
                        id: barCategoryAxis
                        categories: ["V", "A"] // 定义两个类别
                        labelsVisible: false // 我们不需要显示 "V", "A" 标签
                        gridVisible: false
                    },
                    ValueAxis {
                        id: voltageAxis
                        min: 0
                        max: 500
                        tickCount: 6
                        labelsColor: "#A0B0C0" // 浅蓝色字体
                        labelFormat: "%.1f"   // 显示一位小数
                        gridVisible: false
                        lineVisible: true // 显示轴线
                    },

                    ValueAxis {
                        id: currentAxis
                        min: 0
                        max: 250
                        tickCount: 6
                        labelsColor: "#A0B0C0" // 浅蓝色字体
                        labelFormat: "%.1f"   // 显示一位小数
                        gridVisible: false
                        lineVisible: true // 显示轴线
                    }
                ]

                // 电压的堆叠条形图系列
                StackedBarSeries {
                    id: voltageSeries
                    axisX: barCategoryAxis
                    axisY: voltageAxis // 关联到左边的电压Y轴
                    // 设置柱子的宽度，1.0为最大宽度，可以调整这个值
                    barWidth: 0.9
                    BarSet {
                        label: "Voltage Base"
                        values: [380] // 第一个柱子的第一个分段值
                        color: "#008080" // 蓝绿色
                    }
                    BarSet {
                        label: "Voltage Warning"
                        values: [40] // 第一个柱子的第二个分段值
                        color: "orange"
                    }
                    BarSet {
                        label: "Voltage Danger"
                        values: [30] // 第一个柱子的第三个分段值
                        color: "red"
                    }
                }

                // 电流的堆叠条形图系列
                StackedBarSeries {
                    id: currentSeries
                    axisX: barCategoryAxis
                    axisYRight: currentAxis // 关联到右边的电流Y轴
                    // 设置柱子的宽度，1.0为最大宽度，可以调整这个值
                    barWidth: 0.9
                    BarSet {
                        label: "Current Base"
                        values: [0, 220] // 第二个柱子的第一个分段值 (第一个值用0占位)
                        color: "#008080" // 蓝绿色
                    }
                    BarSet {
                        label: "Current Warning"
                        values: [0, 15] // 第二个柱子的第二个分段值
                        color: "orange"
                    }
                    BarSet {
                        label: "Current Danger"
                        values: [0, 10] // 第二个柱子的第三个分段值
                        color: "red"
                    }
                }
            }

            Rectangle{
                id: valueDisplay
                color: Theme.controlBgColor
                anchors{
                    left: parent.left
                    right: parent.right
                    bottom: parent.bottom
                    topMargin: 5
                    leftMargin: 20
                    rightMargin: 20
                    bottomMargin: 5
                }
                height: 30
                RowLayout {
                    anchors.fill: parent
                    Rectangle {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        color: Theme.titleColor
                        Layout.preferredWidth: (parent.width - 30) / 2
                        radius: 12
                        Label {
                            id: voltageLabel
                            anchors.centerIn: parent
                            text: "0.0 V"
                            color: Theme.buttonSelectedTextColor
                            font: Theme.defaultFont
                        }
                    }
                    Item {
                        width: voltageIndicatorWidth
                        Layout.fillHeight: true
                        Label { id: upArrow; anchors.centerIn: parent; text: "▲"; color: "red"; visible: false; font.pointSize: 20 }
                        Label { id: downArrow; anchors.centerIn: parent; text: "▼"; color: "green"; visible: false; font.pointSize: 20 }
                    }
                    Rectangle{
                        color: Theme.titleColor
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        Layout.preferredWidth: (parent.width - 30) / 2
                        radius: 12
                        Label {
                            id: currentLabel
                            anchors.centerIn: parent
                            text: "0.0 A"
                            color: Theme.buttonSelectedTextColor
                            font: Theme.defaultFont
                        }
                    }

                }
            }
        }

        Rectangle{
            color: Theme.controlBgColor
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 60
            RowLayout {
                anchors.fill: parent
                spacing: Theme.subSpacing
                ContinuousActionButton { id: voltageUpButton; labelText: "升压" }
                ContinuousActionButton { id: voltageDownButton; labelText: "降压"; colorWhenOn: "lime" }
            }
        }
    }
}
