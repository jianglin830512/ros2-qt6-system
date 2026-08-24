import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

Rectangle {
    id: root
    Layout.fillWidth: true
    Layout.preferredHeight: 80 // 【修改】为了适配调高的组件，将整体高度由70增加到80
    color: Theme.controlBgColor
    border.color: Theme.highlightColor
    border.width: 2
    radius: 5

    property alias dateString: dateInput.text
    property alias spanDays: spanCombo.currentValue
    property alias circuitId: circuitCombo.currentIndex // 0 for circuit 1, 1 for circuit 2

    signal queryClicked()

    RowLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 20

        // 1. 日期选择
        RowLayout {
            Label { text: "日期:"; color: Theme.textColor; font: Theme.defaultFont }
            Rectangle {
                Layout.preferredWidth: 140
                Layout.preferredHeight: 45 // 【修改】高度调高 25% (原为 36)
                color: "transparent"
                border.color: Theme.highlightColor
                border.width: 2
                radius: 10

                TextInput {
                    id: dateInput
                    anchors.fill: parent; anchors.margins: 5
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    color: Theme.textColor
                    font: Theme.defaultFont
                    text: Qt.formatDateTime(new Date(), "yyyy-MM-dd")
                    readOnly: true // 限制只能通过点击日历修改，防止格式错误

                    MouseArea {
                        anchors.fill: parent
                        onClicked: calendarPopup.open()
                    }
                }
            }
        }

        // 2. 时长选择
        RowLayout {
            Label { text: "时长:"; color: Theme.textColor; font: Theme.defaultFont }
            ComboBox {
                id: spanCombo
                Layout.preferredWidth: 100
                Layout.preferredHeight: 45 // 【修改】高度调高 25%
                model: [1, 2, 3]
                currentIndex: 0
                font: Theme.defaultFont
            }
            Label { text: "天"; color: Theme.textColor; font: Theme.defaultFont }
        }

        // 3. 目标回路选择
        RowLayout {
            Label { text: "目标回路:"; color: Theme.textColor; font: Theme.defaultFont }
            ComboBox {
                id: circuitCombo
                Layout.preferredWidth: 140
                Layout.preferredHeight: 45 // 【修改】高度调高 25%
                model: ["回路 1", "回路 2"]
                currentIndex: 0
                font: Theme.defaultFont
            }
        }

        Item { Layout.fillWidth: true } // 弹簧占位

        // 4. 查询按钮
        StyledButton {
            id: queryBtn
            text: "查 询"
            implicitWidth: 120
            implicitHeight: 45
            onClicked: root.queryClicked()
        }
    }

    // 日历选择弹窗
    Popup {
        id: calendarPopup
        x: 60
        y: 60
        width: 320
        height: 350
        modal: true
        focus: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnPressOutside

        background: Rectangle {
            color: Theme.controlBgColor
            border.color: Theme.highlightColor
            border.width: 2
            radius: 8
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 10
            spacing: 5

            RowLayout {
                Layout.fillWidth: true
                ToolButton {
                    text: "◀"
                    font.bold: true
                    onClicked: {
                        monthGrid.month = monthGrid.month - 1
                        if (monthGrid.month < 0) {
                            monthGrid.month = 11
                            monthGrid.year = monthGrid.year - 1
                        }
                    }
                }
                Label {
                    Layout.fillWidth: true
                    horizontalAlignment: Text.AlignHCenter
                    text: monthGrid.year + "年 " + (monthGrid.month + 1) + "月"
                    font: Theme.defaultFont
                    color: Theme.titleColor
                }
                ToolButton {
                    text: "▶"
                    font.bold: true
                    onClicked: {
                        monthGrid.month = monthGrid.month + 1
                        if (monthGrid.month > 11) {
                            monthGrid.month = 0
                            monthGrid.year = monthGrid.year + 1
                        }
                    }
                }
            }

            DayOfWeekRow {
                locale: monthGrid.locale
                Layout.fillWidth: true
                delegate: Text {
                    text: model.shortName
                    font: Theme.smallLabelFont
                    horizontalAlignment: Text.AlignHCenter
                    color: Theme.orange
                }
            }

            MonthGrid {
                id: monthGrid
                locale: Qt.locale("zh_CN")
                month: new Date().getMonth()
                year: new Date().getFullYear()
                Layout.fillWidth: true
                Layout.fillHeight: true

                delegate: Rectangle {
                    width: monthGrid.width / 7
                    height: monthGrid.height / 6
                    color: "transparent"

                    Rectangle {
                        anchors.centerIn: parent
                        width: Math.min(parent.width, parent.height) * 0.8
                        height: width
                        radius: 5
                        color: (dateInput.text === Qt.formatDateTime(model.date, "yyyy-MM-dd")) ? Theme.highlightColor : "transparent"

                        Text {
                            anchors.centerIn: parent
                            text: model.day
                            color: model.month === monthGrid.month ?
                                (parent.color === Theme.highlightColor ? "white" : Theme.textColor) :
                                Theme.statusDisabledColor
                            font: Theme.defaultFont
                        }
                    }
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            if (model.month === monthGrid.month) {
                                dateInput.text = Qt.formatDateTime(model.date, "yyyy-MM-dd");
                                calendarPopup.close();
                            }
                        }
                    }
                }
            }
        }
    }
}
