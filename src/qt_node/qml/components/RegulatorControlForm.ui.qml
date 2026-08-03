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

    // --- Chart Aliases ---
    property alias voltageAxis: voltageAxis
    property alias currentAxis: currentAxis
    property alias volBaseSet: volBaseSet
    property alias volWarnSet: volWarnSet
    property alias volDangerSet: volDangerSet
    property alias curBaseSet: curBaseSet
    property alias curWarnSet: curWarnSet
    property alias curDangerSet: curDangerSet

    property int voltageIndicatorWidth: 30

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        Rectangle {
            id: topTitleArea  // 【修改1】：分配一个 ID 供底部遮罩定位
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
                        color: Theme.errorColor
                        visible: false
                        font: Theme.defaultFont
                    }
                }
            }
        }

        Rectangle {
            color: Theme.controlBgColor
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.minimumHeight: 60 // 调压器原本就有 60 的最低限制，所以这里的按钮不会溢出
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

                BarCategoryAxis {
                    id: barCategoryAxis
                    categories: ["V", "A"]
                    labelsVisible: false
                    gridVisible: false
                }
                ValueAxis {
                    id: voltageAxis
                    min: 0; max: 450
                    tickCount: 6
                    labelsColor: Theme.axisLabelColor
                    labelFormat: "%.0f"
                    gridVisible: false
                    lineVisible: true
                }
                ValueAxis {
                    id: currentAxis
                    min: 0; max: 450
                    tickCount: 6
                    labelsColor: Theme.axisLabelColor
                    labelFormat: "%.0f"
                    gridVisible: false
                    lineVisible: true
                }

                StackedBarSeries {
                    id: voltageSeries
                    axisX: barCategoryAxis
                    axisY: voltageAxis
                    barWidth: 0.8
                    BarSet { id: volBaseSet; label: "V Base"; values: [0]; color: Theme.chartBaseColor }
                    BarSet { id: volWarnSet; label: "V Warning"; values: [0]; color: Theme.chartWarnColor }
                    BarSet { id: volDangerSet; label: "V Danger"; values: [0]; color: Theme.chartDangerColor }
                }

                StackedBarSeries {
                    id: currentSeries
                    axisX: barCategoryAxis
                    axisYRight: currentAxis
                    barWidth: 0.8
                    BarSet { id: curBaseSet; label: "A Base"; values: [0, 0]; color: Theme.chartBaseColor }
                    BarSet { id: curWarnSet; label: "A Warning"; values: [0, 0]; color: Theme.chartWarnColor }
                    BarSet { id: curDangerSet; label: "A Danger"; values: [0, 0]; color: Theme.chartDangerColor }
                }
            }

            Rectangle {
                id: valueDisplay
                color: Theme.controlBgColor
                anchors {
                    left: parent.left; right: parent.right; bottom: parent.bottom
                    topMargin: 5; leftMargin: 20; rightMargin: 20; bottomMargin: 5
                }
                height: 30
                RowLayout {
                    anchors.fill: parent
                    Rectangle {
                        Layout.fillWidth: true; Layout.fillHeight: true
                        color: "transparent"
                        border.color: Theme.highlightColor
                        border.width: 2
                        Layout.preferredWidth: (parent.width - 30) / 2
                        radius: 12
                        Label {
                            id: voltageLabel
                            anchors.centerIn: parent
                            text: "0.0 V"
                            color: Theme.valueColor
                            font: Theme.defaultFont
                        }
                    }
                    Item {
                        width: voltageIndicatorWidth
                        Layout.fillHeight: true
                        Label { id: upArrow; anchors.centerIn: parent; text: "▲"; color: Theme.arrowUpColor; visible: false; font.pointSize: 20 }
                        Label { id: downArrow; anchors.centerIn: parent; text: "▼"; color: Theme.arrowDownColor; visible: false; font.pointSize: 20 }
                    }
                    Rectangle{
                        color: Theme.titleColor
                        Layout.fillWidth: true; Layout.fillHeight: true
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

        Rectangle {
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
        // 【修改2】：用动态绑定的方式精准推开标题，取代原本硬编码的 50px
        anchors.topMargin: topTitleArea.height + 15
        radius: root.radius
        statusText: "非手动模式"
        visible: false
        overlayColor: Theme.blockerOverlayColor
    }
}
