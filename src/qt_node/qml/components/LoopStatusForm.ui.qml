import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

Rectangle {
    id: root
    implicitWidth: 350 // 给一个合理的最小宽度
    color: "transparent"
    border.color: Theme.highlightColor
    border.width: 1
    Layout.fillWidth: true
    Layout.fillHeight: true

    // --- Aliases for logic file ---
    property alias enableLabel: enableLabel
    property alias titleLabel: titleLabel
    property alias statusLabel: statusLabel
    property alias setCurrentLabel: setCurrentLabel
    property alias measureCurrentLabel: measureCurrentLabel
    property string heatSetValue: "0"
    property string heatRemainValue: "0"
    property string cycleSetValue: "0"
    property string cycleRemainValue: "0"
    property alias tempRepeater: tempRepeater
    property alias closeBreakerButton: closeBreakerButton
    property alias openBreakerButton: openBreakerButton

    ColumnLayout {
        anchors.fill: parent; anchors.margins: 10; spacing: 8

        Item {
            id: titleArea
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 50
            RowLayout {
                anchors.fill: parent
                Item{
                    Layout.fillWidth: true
                    Text {
                        anchors.centerIn: parent
                        id: enableLabel
                        text: "停用"
                        font: Theme.defaultFont
                        color: "red"
                    }
                }
                Item{
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredHeight: 40
                    Text {
                        anchors.centerIn: parent
                        id: titleLabel
                        text: "回路"
                        font: Theme.subjectFont
                        color: Theme.orange
                    }
                }
                Item{
                    Layout.fillWidth: true
                    Text {
                        anchors.centerIn: parent
                        id: statusLabel
                        text: "状态"
                        font: Theme.defaultFont
                        color: "red"
                    }
                }
            }
        }

        Item {
            id: currentSettingPanle
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 50
            RowLayout{
                anchors.fill: parent
                Label {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    text: "设定电流:"
                    color: Theme.titleColor
                    font: Theme.largeLabelFont
                }
                Rectangle { // [修改] 为设定电流值添加边框
                    color: "transparent"
                    border.color: Theme.textColor
                    border.width: 2
                    radius: 5
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredWidth: 120
                    Label {
                        id: setCurrentLabel
                        text: "0"
                        color: Theme.textColor
                        font: Theme.largeLabelFont
                        anchors.centerIn: parent
                    }
                }
                Label {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredWidth: 50
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    text: "A"
                    color: Theme.titleColor
                    font: Theme.largeLabelFont
                }
            }
        }

        Item {
            id: currentValuePanle
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 50
            RowLayout{
                anchors.fill: parent
                Label {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    text: "测量电流:"
                    color: Theme.titleColor
                    font: Theme.largeLabelFont
                }
                Item {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredWidth: 120
                    Label {
                        id: measureCurrentLabel
                        text: "0"
                        color: Theme.textColor
                        font: Theme.largeLabelFont
                        anchors.centerIn: parent
                    }
                }
                Label {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredWidth: 50
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    text: "A"
                    color: Theme.titleColor
                    font: Theme.largeLabelFont
                }
            }
        }
        Item {
            id: breakerButtonPanle
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 60
            RowLayout { // 2 button
                anchors.centerIn: parent
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
        Item {
            id: timePanle
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 100
            GridLayout {
                anchors.fill: parent
                columns: 3
                rowSpacing: 5
                ValueAndUnit{
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredWidth: 100
                    title: "加热设定:"
                    value: heatSetValue
                    unit: "min"
                }
                Item {
                    Layout.preferredWidth: 20
                }
                ValueAndUnit{
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredWidth: 70
                    title: "剩余:"
                    value: heatRemainValue
                    unit: "min"
                }
                ValueAndUnit{
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    title: "循环设定:"
                    value: cycleSetValue
                    unit: "次"
                }
                Item {
                    Layout.preferredWidth: 20
                }
                ValueAndUnit{
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    title: "剩余:"
                    value: cycleRemainValue
                    unit: "次"
                }
            }
        }

        Label { text: "温度"; color: Theme.orange; font: Theme.defaultFont;Layout.alignment: Layout.Center }

        Item {
            id: temperaturePanle
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 500
            GridLayout {
                anchors.fill: parent
                rows: 8
                flow: GridLayout.TopToBottom
                Repeater {
                    id: tempRepeater
                    delegate: ValueAndUnit {
                        Layout.preferredWidth: (parent.width - 30) / 2
                        Layout.fillHeight: true
                        title: modelData.titleName
                        value: modelData.value
                        unit: "℃"
                    }
                }
            }
        }
    }
}
