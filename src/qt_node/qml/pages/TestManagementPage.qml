import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import QtQuick.Dialogs
import qt_node 1.0
import qt.theme 1.0
import "../components"

TestManagementPageForm {
    id: page

    property int currentPage: 1
    property int totalPages: 1
    property int pageSize: 20

    property int selectedRecordId: -1
    property var selectedRecordPoints: []

    // 【修改点】：追加了试验ID
    property var tableHeaders: [
        {title: "试验ID", weight: 0.8},
        {title: "回路ID", weight: 0.5},
        {title: "起始日期", weight: 1.0},
        {title: "结束日期", weight: 1.0},
        {title: "电缆名称", weight: 1.5},
        {title: "备注", weight: 1.5},
        {title: "操作", weight: 1.0}
    ]
    property real totalWeight: 7.3
    property var tableRows: []

    // 动态取宽和高
    tableContentWidth: Math.max(1200, tableListView.width)
    property real dynamicRowHeight: Math.max(30, tableListView.height / page.pageSize)

    onVisibleChanged: {
        if (visible) { loadData(); }
    }

    function loadData() {
        // 请求全部 circuit_id = 0
        rosProxy.listTestRecords(page.searchInput.text, page.currentPage, page.pageSize, 0);
    }

    Repeater {
        parent: page.headerRowLayout
        model: page.tableHeaders
        delegate: Rectangle {
            width: (modelData.weight / page.totalWeight) * page.tableContentWidth
            height: 50
            color: "transparent"; border.color: Theme.gridLineColor; border.width: 1
            Text { anchors.centerIn: parent; text: modelData.title; color: Theme.buttonSelectedTextColor; font: Theme.defaultFont }
        }
    }

    tableListView.model: page.tableRows
    tableListView.delegate: Item {
        width: page.tableContentWidth
        height: page.dynamicRowHeight
        property var rowData: modelData

        Rectangle {
            anchors.fill: parent
            color: (rowData.id === page.selectedRecordId) ? Theme.tableHoverColor :
                                                            (ma.containsMouse ? Theme.tableHoverColor : (index % 2 === 0 ? "transparent" : Theme.tableAlternateColor))
            border.color: (rowData.id === page.selectedRecordId) ? Theme.highlightColor : "transparent"
            border.width: (rowData.id === page.selectedRecordId) ? 2 : 0
        }

        MouseArea {
            id: ma; anchors.fill: parent; hoverEnabled: true
            onClicked: {
                page.selectedRecordId = rowData.id;
                page.selectedRecordPoints = rowData.temp_points;
                // 点击时更新上方提示信息面板的数据
                page.infoCircuitId.text = rowData.circuit_id;
                page.infoStartDate.text = rowData.start_date;
                page.infoEndDate.text = rowData.end_date;
                page.infoCableName.text = rowData.cable_name;
                page.infoRemarks.text = rowData.remarks;
            }
        }

        Row {
            height: parent.height
            // 新增试验 ID 渲染列
            Rectangle { width: (page.tableHeaders[0].weight/page.totalWeight)*page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: rowData.id; color: Theme.valueColor; font: Theme.smallLabelFont } }
            Rectangle { width: (page.tableHeaders[1].weight/page.totalWeight)*page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: rowData.circuit_id; color: Theme.valueColor; font: Theme.smallLabelFont } }
            Rectangle { width: (page.tableHeaders[2].weight/page.totalWeight)*page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: rowData.start_date; color: Theme.valueColor; font: Theme.smallLabelFont } }
            Rectangle { width: (page.tableHeaders[3].weight/page.totalWeight)*page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: rowData.end_date; color: Theme.valueColor; font: Theme.smallLabelFont } }
            Rectangle { width: (page.tableHeaders[4].weight/page.totalWeight)*page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: rowData.cable_name; color: Theme.valueColor; font: Theme.smallLabelFont } }
            Rectangle { width: (page.tableHeaders[5].weight/page.totalWeight)*page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: rowData.remarks; color: Theme.valueColor; font: Theme.smallLabelFont; elide: Text.ElideRight; width: parent.width-10; horizontalAlignment: Text.AlignHCenter } }

            Rectangle {
                width: (page.tableHeaders[6].weight/page.totalWeight)*page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Row {
                    anchors.centerIn: parent; spacing: 10
                    Rectangle {
                        width: 50; height: 28; color: editMa.pressed ? Theme.buttonSelectedGradientStart : Theme.highlightColor; radius: 4
                        Text { anchors.centerIn: parent; text: "修改"; color: "white"; font: Theme.smallLabelFont }
                        MouseArea { id: editMa; anchors.fill: parent; onClicked: {
                                editDialog.loadRecord(rowData); editDialog.open();
                            }}
                    }
                    Rectangle {
                        width: 50; height: 28; color: delMa.pressed ? Theme.buttonDangerPressedColor : Theme.buttonDangerColor; radius: 4
                        Text { anchors.centerIn: parent; text: "删除"; color: "white"; font: Theme.smallLabelFont }
                        MouseArea { id: delMa; anchors.fill: parent; onClicked: {
                                deleteConfirmDialog.targetId = rowData.id; deleteConfirmDialog.open();
                            }}
                    }
                }
            }
        }
    }

    btnSearch.onClicked: { page.currentPage = 1; loadData(); }
    btnPrevPage.onClicked: { if (page.currentPage > 1) { page.currentPage--; loadData(); } }
    btnNextPage.onClicked: { if (page.currentPage < page.totalPages) { page.currentPage++; loadData(); } }
    btnAdd.onClicked: { editDialog.loadRecord(null); editDialog.open(); }
    btnExport.onClicked: { exportDialog.open() }

    // 将 40个温度点 注入到底部的详情容器中
    GridLayout {
        parent: page.detailsContainer
        anchors.fill: parent
        columns: 9
        rowSpacing: 5
        columnSpacing: 10

        // 表头
        Item { Layout.preferredWidth: 40;Layout.preferredHeight: 40 }
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
            model: 45
            Rectangle {
                id: detailCellItem
                Layout.fillWidth: true
                Layout.preferredHeight: 40
                border.color: Theme.buttonBorderColor
                border.width: !isRowHeader ? 1 : 0
                color: "transparent"

                property bool isRowHeader: index % 9 === 0
                property int dataIndex: index - Math.floor(index / 9) - 1

                Label {
                    anchors.fill: parent
                    text: "采集卡" + (Math.floor(index / 9) + 1)
                    color: Theme.textColor
                    font: Theme.defaultFont
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignRight
                    padding: 5
                    visible: detailCellItem.isRowHeader
                }

                TextInput {
                    id: ptInputRO
                    anchors.fill: parent
                    anchors.margins: 4
                    font: Theme.defaultFont
                    color: Theme.textColor
                    verticalAlignment: Text.AlignVCenter
                    horizontalAlignment: Text.AlignHCenter
                    clip: true
                    readOnly: true
                    selectByMouse: true
                    visible: !detailCellItem.isRowHeader
                    text: {
                        if (detailCellItem.visible && page.selectedRecordPoints && page.selectedRecordPoints.length > detailCellItem.dataIndex) {
                            let val = page.selectedRecordPoints[detailCellItem.dataIndex];
                            return (val !== undefined && val !== null) ? val.toString() : "";
                        }
                        return "";
                    }

                    ToolTip.visible: maDetails.containsMouse && ptInputRO.text !== ""
                    ToolTip.text: ptInputRO.text
                    ToolTip.delay: 500

                    MouseArea {
                        id: maDetails
                        anchors.fill: parent
                        hoverEnabled: true
                        acceptedButtons: Qt.NoButton
                    }
                }
            }
        }
    }

    FileDialog {
        id: exportDialog
        title: "导出试验记录"
        fileMode: FileDialog.SaveFile
        nameFilters: ["CSV 格式文件 (*.csv)"]
        defaultSuffix: "csv"
        onAccepted: { rosProxy.exportTestRecords(selectedFile.toString()) }
    }

    Connections {
        target: rosProxy
        function onTestRecordsListed(circuitId, result) {
            // TestManagementPage 这里只捕获无过滤的查询 (circuitId = 0)
            if (circuitId === 0) {
                if (result.success) {
                    page.totalPages = result.total_pages; page.currentPage = result.current_page;
                    page.pageLabel.text = result.current_page + " / " + result.total_pages;
                    page.tableRows = result.records;

                    // 刷新数据后，重置详情选择和提示
                    page.selectedRecordId = -1;
                    page.selectedRecordPoints = [];
                    page.infoCircuitId.text = "";
                    page.infoStartDate.text = "";
                    page.infoEndDate.text = "";
                    page.infoCableName.text = "";
                    page.infoRemarks.text = "";
                } else { messagePopup.isError=true; messagePopup.message="查询失败"; messagePopup.open(); }
            }
        }
        function onTestRecordSaveResult(success, message) {
            if (success) { loadData(); } else { messagePopup.isError=true; messagePopup.message="保存失败:"+message; messagePopup.open(); }
        }
        function onTestRecordDeleteResult(success, message) {
            if (success) { loadData(); } else { messagePopup.isError=true; messagePopup.message="删除失败:"+message; messagePopup.open(); }
        }
        function onExportTestRecordResult(success, message) {
            messagePopup.isError = !success; messagePopup.message = message; messagePopup.open();
        }
    }

    TestRecordEditDialog { id: editDialog }

    Dialog {
        id: deleteConfirmDialog
        property int targetId: -1
        title: "确认删除"
        modal: true; anchors.centerIn: Overlay.overlay; width: 350
        background: Rectangle { color: Theme.controlBgColor; border.color: Theme.highlightColor; border.width: 2; radius: 8 }
        header: Label { text: title; color: Theme.errorColor; font: Theme.subjectFont; padding: 15; horizontalAlignment: Text.AlignHCenter }
        contentItem: Label { text: "确定要永久删除该记录吗？"; color: Theme.textColor; font: Theme.defaultFont; horizontalAlignment: Text.AlignHCenter; padding: 20 }
        footer: DialogButtonBox {
            alignment: Qt.AlignHCenter; padding: 15
            Button { text: "确定"; onClicked: { rosProxy.deleteTestRecord(deleteConfirmDialog.targetId); deleteConfirmDialog.close(); } }
            Button { text: "取消"; onClicked: deleteConfirmDialog.close() }
        }
    }
    MessagePopup { id: messagePopup }
}
