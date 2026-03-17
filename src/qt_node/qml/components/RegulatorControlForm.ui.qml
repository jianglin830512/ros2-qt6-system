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
    property alias isBlocked: inputBlocker.visible

    // --- Chart Aliases 暴露给逻辑层 ---
    property alias voltageAxis: voltageAxis
    property alias currentAxis: currentAxis
    property alias volBaseSet: volBaseSet
    property alias volWarnSet: volWarnSet
    property alias volDangerSet: volDangerSet
    property alias curBaseSet: curBaseSet
    property alias curWarnSet: curWarnSet
    property alias curDangerSet: curDangerSet

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

        // --- Chart ---
        Rectangle {
            Layout.fillWidth: true
            Layout.minimumHeight: 200
            Layout.fillHeight: true
            color: "transparent"
            border.color: Theme.highlightColor
            ChartView {
                id: chartView
                anchors.fill: parent
                anchors.bottomMargin: 15
                antialiasing: true
                legend.visible: false
                backgroundColor: Theme.controlBgColor

                // 移除原有的 axes: [ ... ] 写法，直接嵌套！
                BarCategoryAxis {
                    id: barCategoryAxis
                    categories: ["V", "A"]
                    labelsVisible: false
                    gridVisible: false
                }
                ValueAxis {
                    id: voltageAxis
                    min: 0
                    max: 450 // 电压固定为 450V
                    tickCount: 6
                    labelsColor: "#A0B0C0"
                    labelFormat: "%.0f"
                    gridVisible: false
                    lineVisible: true
                }
                ValueAxis {
                    id: currentAxis
                    min: 0
                    max: 450 // 默认450，逻辑层会根据主辅重新赋值
                    tickCount: 6
                    labelsColor: "#A0B0C0"
                    labelFormat: "%.0f"
                    gridVisible: false
                    lineVisible: true
                }

                // 电压的堆叠条形图
                StackedBarSeries {
                    id: voltageSeries
                    axisX: barCategoryAxis
                    axisY: voltageAxis
                    barWidth: 0.8
                    BarSet { id: volBaseSet; label: "V Base"; values: [0]; color: "#008080" }
                    BarSet { id: volWarnSet; label: "V Warning"; values: [0]; color: "orange" }
                    BarSet { id: volDangerSet; label: "V Danger"; values: [0]; color: "red" }
                }

                // 电流的堆叠条形图
                StackedBarSeries {
                    id: currentSeries
                    axisX: barCategoryAxis
                    axisYRight: currentAxis
                    barWidth: 0.8
                    // 注意：电流是第二列(类别"A")，所以用 [0, 0] 占位，之后在索引 1 处修改值
                    BarSet { id: curBaseSet; label: "A Base"; values: [0, 0]; color: "#008080" }
                    BarSet { id: curWarnSet; label: "A Warning"; values: [0, 0]; color: "orange" }
                    BarSet { id: curDangerSet; label: "A Danger"; values: [0, 0]; color: "red" }
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

    InputBlocker {
        id: inputBlocker
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.top: parent.top
        anchors.topMargin: 50
        radius: root.radius
        statusText: "非手动模式"
        visible: false
    }
}
