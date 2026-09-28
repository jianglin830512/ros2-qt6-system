import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

Dialog {
    id: root
    property int recordId: -1
    property var cables: []
    property string activeInput: ""
    title: recordId < 0 ? "新增试验记录" : "修改试验记录"
    modal: true
    anchors.centerIn: Overlay.overlay
    width: 1400  // 加宽弹窗以充分展开下方布局
    height: 700

    background: Rectangle {
        color: Theme.controlBgColor
        border.color: Theme.highlightColor
        border.width: 2
        radius: 8
    }

    header: Label {
        text: root.title
        color: Theme.titleColor
        font: Theme.subjectFont
        padding: 15
        horizontalAlignment: Text.AlignHCenter
    }

    function loadRecord(data) {
        // 加载最新电缆用于下拉菜单
        rosProxy.listCables("", 1, 9999, 1, true);

        if (data) {
            recordId = data.id;
            circuitCombo.currentIndex = data.circuit_id - 1;
            startDateInput.text = data.start_date;
            endDateInput.text = data.end_date;
            remarksInput.text = data.remarks;
            // 回显 40 个测温点
            for(let i = 0; i < 45; i++) {
                let item = pointRepeater.itemAt(i);
                if (!item.isRowHeader) {
                    item.textValue = data.temp_points[item.dataIndex] || "";
                }
            }
            // 延时选中下拉框，因为 cables 可能还没返回
            root.targetCableId = data.cable_id;
        } else {
            recordId = -1;
            circuitCombo.currentIndex = 0;
            root.targetCableId = -1;
            startDateInput.text = Qt.formatDateTime(new Date(), "yyyy-MM-dd");
            endDateInput.text = Qt.formatDateTime(new Date(), "yyyy-MM-dd");
            remarksInput.text = "";
            for(let i = 0; i < 45; i++) {
                let item = pointRepeater.itemAt(i);
                if (!item.isRowHeader) {
                    item.textValue = "";
                }
            }
        }
    }

    property int targetCableId: -1
    Connections {
        target: rosProxy
        function onCablesListed(result) {
            if (result.success) {
                root.cables = result.cables;
                cableCombo.model = root.cables;
                if(root.targetCableId !== -1) {
                    let idx = root.cables.findIndex(c => c.id === root.targetCableId);
                    if(idx >= 0) cableCombo.currentIndex = idx;
                }
            }
        }
    }

    ColumnLayout {
        anchors.fill:parent
        anchors.margins: 20
        spacing: 15

        // 基本信息区域，8 列对应 4个项目（每项包含Label和输入控件）
        GridLayout {
            columns: 8
            rowSpacing: 10
            columnSpacing: 30
            Layout.fillWidth: true

            // --- 第一行 ---
            Label { text: "回路ID:"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignRight | Qt.AlignVCenter }
            ComboBox { id: circuitCombo; model: ["1", "2"]; font: Theme.defaultFont; Layout.preferredWidth: 100; Layout.preferredHeight: 36 }

            Label { text: "电缆:"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignRight | Qt.AlignVCenter }
            ComboBox { id: cableCombo; textRole: "name"; font: Theme.defaultFont; Layout.preferredWidth: 200; Layout.minimumWidth: 120; Layout.preferredHeight: 36 }

            // 【修复核心】设定首选宽度，防止挤压导致文字重叠
            Label { text: "起始日期:"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignRight | Qt.AlignVCenter }
            Rectangle {
                Layout.preferredWidth: 200; Layout.preferredHeight: 36; border.color: Theme.buttonBorderColor; border.width: 1; color: "transparent"
                TextInput {
                    id: startDateInput; anchors.fill: parent; anchors.margins:5; font: Theme.defaultFont; verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter
                    readOnly: true; clip: true
                    MouseArea { anchors.fill: parent; onClicked: { root.activeInput = "start"; calendarPopup.open(); } }
                }
            }

            Label { text: "结束日期:"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignRight | Qt.AlignVCenter }
            Rectangle {
                Layout.preferredWidth: 200; Layout.preferredHeight: 36; border.color: Theme.buttonBorderColor; border.width: 1; color: "transparent"
                TextInput {
                    id: endDateInput; anchors.fill: parent; anchors.margins:5; font: Theme.defaultFont; verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter
                    readOnly: true; clip: true
                    MouseArea { anchors.fill: parent; onClicked: { root.activeInput = "end"; calendarPopup.open(); } }
                }
            }

            // --- 第二行 ---
            Label { text: "备注:"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignRight | Qt.AlignVCenter }
            Rectangle {
                Layout.fillWidth: true; Layout.preferredHeight: 36; border.color: Theme.buttonBorderColor; border.width: 1; color: "transparent"
                Layout.columnSpan: 7 // 占满后面所有的格子
                TextInput {
                    id: remarksInput
                    anchors.fill: parent
                    anchors.margins: 5
                    font: Theme.defaultFont
                    verticalAlignment: Text.AlignVCenter
                    clip: true
                }
            }
        }

        // 40路测温点
        Label { text: "测温点位置设定 (每通道最大50字)"; color: Theme.orange; font: Theme.defaultFont; Layout.topMargin: 15 }

        GridLayout {
            columns: 9
            rowSpacing: 10
            columnSpacing: 10
            Layout.fillWidth: true

            // 表头
            Item { Layout.preferredWidth: 40 }
            Repeater {
                model: 8
                Label {
                    Layout.preferredWidth: 80
                    text: (index + 1).toString()
                    color: Theme.textColor
                    font: Theme.defaultFont
                    horizontalAlignment: Text.AlignHCenter
                    Layout.fillWidth: true
                }
            }

            // 单元格数据 (5行 x 9列 = 45个元素)
            Repeater {
                id: pointRepeater
                model: 45

                Rectangle {
                    id: cellItem
                    Layout.fillWidth: true
                    Layout.preferredHeight: 40 // 设置更高
                    border.color: Theme.buttonBorderColor
                    border.width: !isRowHeader ? 1 : 0
                    color: "transparent"

                    property bool isRowHeader: index % 9 === 0
                    property int dataIndex: index - Math.floor(index / 9) - 1
                    property alias textValue: ptInput.text

                    Label {
                        anchors.fill: parent
                        text: "采集卡" + (Math.floor(index / 9) + 1)
                        color: Theme.textColor
                        font: Theme.defaultFont
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: Text.AlignRight
                        padding: 5
                        visible: cellItem.isRowHeader
                    }

                    TextInput {
                        id: ptInput
                        anchors.fill: parent
                        anchors.margins: 4
                        font: Theme.defaultFont // 字体放大
                        color: Theme.textColor
                        maximumLength: 50
                        verticalAlignment: Text.AlignVCenter
                        horizontalAlignment: Text.AlignHCenter
                        clip: true
                        visible: !cellItem.isRowHeader
                    }
                }
            }
        }
    }


    footer: DialogButtonBox {
        alignment: Qt.AlignHCenter
        background: Rectangle { color: "transparent" }
        padding: 15

        Button {
            text: "保存"
            implicitWidth: 100; implicitHeight: 40
            background: Rectangle { color: Theme.statusOkColor; radius: 5 }
            contentItem: Text { text: parent.text; color: "white"; font: Theme.buttonFont; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
            onClicked: {
                if(cableCombo.currentIndex < 0) return;
                let c = root.cables[cableCombo.currentIndex];
                let tps = [];
                for(let i = 0; i < 45; i++) {
                    let item = pointRepeater.itemAt(i);
                    if (!item.isRowHeader) {
                        tps.push(item.textValue);
                    }
                }

                var recordMap = {
                    "id": root.recordId,
                    "circuit_id": parseInt(circuitCombo.currentText),
                    "start_date": startDateInput.text,
                    "end_date": endDateInput.text,
                    "cable_id": c.id,
                    "cable_name": c.name,
                    "remarks": remarksInput.text,
                    "temp_points": tps
                };
                rosProxy.saveTestRecord(recordMap);
                root.close();
            }
        }

        Button {
            text: "取消"
            implicitWidth: 100; implicitHeight: 40
            background: Rectangle { color: Theme.buttonHoverColor; radius: 5 }
            contentItem: Text { text: parent.text; color: Theme.textColor; font: Theme.buttonFont; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
            onClicked: root.close()
        }
    }

    // 日期选择弹窗
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
                        if (monthGrid.month < 0) { monthGrid.month = 11; monthGrid.year = monthGrid.year - 1; }
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
                        if (monthGrid.month > 11) { monthGrid.month = 0; monthGrid.year = monthGrid.year + 1; }
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
                            let dText = root.activeInput === "start" ? startDateInput.text : endDateInput.text;
                            return (dText === Qt.formatDateTime(model.date, "yyyy-MM-dd")) ? Theme.highlightColor : "transparent"
                        }

                        Text {
                            anchors.centerIn: parent
                            text: model.day
                            color: model.month === monthGrid.month ?
                                       (parent.color === Theme.highlightColor ? "white" : Theme.textColor) : Theme.statusDisabledColor
                            font: Theme.defaultFont
                        }
                    }
                    MouseArea {
                        anchors.fill: parent
                        onClicked: {
                            if (model.month === monthGrid.month) {
                                if (root.activeInput === "start") {
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
}
