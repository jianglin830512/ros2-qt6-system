import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick.Dialogs
import qt_node 1.0
import qt.theme 1.0
import "../components"

DataExportPageForm {
    id: page

    property bool isExporting: false
    property string activeInput: "" // 标记当前正在选择 "start" 还是 "end" 日期

    Component.onCompleted: {
        startDateInput.text = Qt.formatDateTime(new Date(), "yyyy-MM-dd")
        endDateInput.text = Qt.formatDateTime(new Date(), "yyyy-MM-dd")
    }

    startMa.onClicked: {
        if (isExporting) return;
        activeInput = "start"
        calendarPopup.open()
    }

    endMa.onClicked: {
        if (isExporting) return;
        activeInput = "end"
        calendarPopup.open()
    }

    exportBtn.onClicked: {
        if (isExporting) return;

        let start = new Date(startDateInput.text);
        let end = new Date(endDateInput.text);
        if (start > end) {
            messagePopup.isError = true;
            messagePopup.message = "结束日期不能早于起始日期！";
            messagePopup.open();
            return;
        }

        fileDialog.open()
    }

    // 系统原生的保存对话框，强制拓展名为 csv
    FileDialog {
        id: fileDialog
        title: "选择导出路径"
        fileMode: FileDialog.SaveFile
        nameFilters: ["CSV 格式文件 (*.csv)"]
        defaultSuffix: "csv"
        onAccepted: {
            page.isExporting = true
            exportBtn.enabled = false
            progressBar.value = 0
            progressLabel.text = "0%"
            // Circuit 下拉菜单 0 代表回路1，1 代表回路2
            rosProxy.exportData(startDateInput.text, endDateInput.text, circuitCombo.currentIndex + 1, selectedFile.toString())
        }
    }

    Connections {
        target: rosProxy
        function onExportProgressChanged(percent) {
            progressBar.value = percent
            progressLabel.text = percent + "%"
        }
        function onExportResult(success, message) {
            page.isExporting = false
            exportBtn.enabled = true
            progressBar.value = 100
            progressLabel.text = "100%"

            messagePopup.isError = !success
            messagePopup.message = message
            messagePopup.open()
        }
    }

    // 复用高颜值的日历弹窗
    Popup {
        id: calendarPopup
        x: (parent.width - width) / 2
        y: (parent.height - height) / 2
        width: 320
        height: 350
        modal: true
        focus: true

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
                        color: {
                            let dText = page.activeInput === "start" ? startDateInput.text : endDateInput.text;
                            return (dText === Qt.formatDateTime(model.date, "yyyy-MM-dd")) ? Theme.highlightColor : "transparent"
                        }

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
                                if (page.activeInput === "start") {
                                    startDateInput.text = Qt.formatDateTime(model.date, "yyyy-MM-dd");
                                } else {
                                    endDateInput.text = Qt.formatDateTime(model.date, "yyyy-MM-dd");
                                }
                                calendarPopup.close();
                            }
                        }
                    }
                }
            }
        }
    }

    MessagePopup { id: messagePopup }
}
