import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0
import "../components"

Item {
    id: root

    property alias startDateInput: startDateInput
    property alias endDateInput: endDateInput
    property alias startMa: startMa
    property alias endMa: endMa
    property alias circuitCombo: circuitCombo
    property alias exportBtn: exportBtn
    property alias progressBar: progressBar
    property alias progressLabel: progressLabel

    ColumnLayout {
        anchors.centerIn: parent
        width: 650
        spacing: 25

        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 580
            color: Theme.controlBgColor
            border.color: Theme.highlightColor
            border.width: 2
            radius: 8

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 40
                spacing: 30

                Label {
                    text: "历 史 数 据 导 出"
                    font: Theme.titleFont
                    color: Theme.titleColor
                    Layout.alignment: Qt.AlignHCenter
                }

                GridLayout {
                    columns: 2
                    rowSpacing: 25
                    columnSpacing: 20
                    Layout.alignment: Qt.AlignHCenter

                    Label { text: "起始日期:"; font: Theme.defaultFont; color: Theme.textColor; Layout.alignment: Qt.AlignRight }
                    Rectangle {
                        Layout.preferredWidth: 240; Layout.preferredHeight: 45
                        color: "transparent"; border.color: Theme.highlightColor; border.width: 2; radius: 5
                        TextInput {
                            id: startDateInput
                            anchors.fill: parent; anchors.margins: 5
                            verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter
                            color: Theme.textColor; font: Theme.defaultFont; readOnly: true
                            MouseArea { id: startMa; anchors.fill: parent }
                        }
                    }

                    Label { text: "结束日期:"; font: Theme.defaultFont; color: Theme.textColor; Layout.alignment: Qt.AlignRight }
                    Rectangle {
                        Layout.preferredWidth: 240; Layout.preferredHeight: 45
                        color: "transparent"; border.color: Theme.highlightColor; border.width: 2; radius: 5
                        TextInput {
                            id: endDateInput
                            anchors.fill: parent; anchors.margins: 5
                            verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter
                            color: Theme.textColor; font: Theme.defaultFont; readOnly: true
                            MouseArea { id: endMa; anchors.fill: parent }
                        }
                    }

                    Label { text: "目标回路:"; font: Theme.defaultFont; color: Theme.textColor; Layout.alignment: Qt.AlignRight }
                    ComboBox {
                        id: circuitCombo
                        Layout.preferredWidth: 240; Layout.preferredHeight: 45
                        font: Theme.defaultFont
                        model: ["回路 1", "回路 2"]
                    }
                }

                Item { Layout.fillHeight: true }

                StyledButton {
                    id: exportBtn
                    text: "选 择 导 出 位 置 并 生 成 CSV"
                    Layout.alignment: Qt.AlignHCenter
                    Layout.preferredWidth: 360
                    Layout.preferredHeight: 50
                }
            }
        }

        // 进度提示区域
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 80
            color: Theme.controlBgColor
            border.color: Theme.highlightColor
            border.width: 2
            radius: 8

            RowLayout {
                anchors.fill: parent
                anchors.margins: 20
                spacing: 15

                Label { text: "导出进度:"; font: Theme.defaultFont; color: Theme.textColor }
                ProgressBar {
                    id: progressBar
                    Layout.fillWidth: true
                    Layout.preferredHeight: 20
                    from: 0
                    to: 100
                    value: 0
                }
                Label {
                    id: progressLabel
                    text: "0%"
                    font: Theme.defaultFont
                    color: Theme.titleColor
                    Layout.preferredWidth: 60
                    horizontalAlignment: Text.AlignRight
                }
            }
        }
    }
}
