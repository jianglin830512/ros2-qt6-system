
import QtQuick
import QtQuick.Controls
import qt_node 1.0
import qt.theme 1.0
import "../components"

TestDataPageForm {
    id: page

    property var tableHeaders: []
    property var tableRows: []

    // 辅助函数：确定列宽，时间列宽一些，其他列较窄
    function getColWidth(index) {
        return index === 0 ? 180 : 120; // 稍微拉宽一点，为了让两行字排版更好看
    }

    // 1. 动态生成表头
    Repeater {
        parent: page.headerRowLayout
        model: page.tableHeaders

        delegate: Rectangle {
            width: getColWidth(index)
            height: 60
            color: "transparent"
            border.color: Theme.gridLineColor
            border.width: 1

            Text {
                anchors.centerIn: parent
                text: modelData
                color: Theme.buttonSelectedTextColor // 使用 Theme 里的白色 (#FFFFFF)
                font: Theme.smallLabelFont
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
            }
        }
    }

    // 2. 数据行视图委托
    tableListView.model: page.tableRows
    tableListView.delegate: Item {
        id: rowItem
        width: innerRow.width
        height: 35
        property var rowData: modelData

        // 鼠标悬停行高亮
        Rectangle {
            anchors.fill: parent
            color: ma.containsMouse ? Theme.tableHoverColor : (index % 2 === 0 ? "transparent" : Theme.tableAlternateColor)
        }

        MouseArea {
            id: ma
            anchors.fill: parent
            hoverEnabled: true
        }

        Row {
            id: innerRow
            height: parent.height

            Repeater {
                model: rowItem.rowData
                delegate: Rectangle {
                    width: getColWidth(index)
                    height: 35
                    color: "transparent"
                    border.color: Theme.gridLineTransparentColor
                    border.width: 1

                    Text {
                        anchors.centerIn: parent
                        text: modelData
                        color: Theme.valueColor
                        font: Theme.smallLabelFont
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }
                }
            }
        }
    }

    // 3. 按钮点击请求数据 (通过监听 queryPanel 发出的信号)
    Connections {
        target: page.queryPanel
        function onQueryClicked() {
            var dateStr = page.queryPanel.dateString;
            var spanHours = parseInt(page.queryPanel.spanDays) * 24;
            var cid = page.queryPanel.circuitId + 1; // 0->1, 1->2

            // 将时间硬编码为 "00:00"
            rosProxy.queryTable(dateStr, "00:00", spanHours, cid);
        }
    }

    // 4. 处理后端发来的数据
    Connections {
        target: rosProxy

        function onTableDataReady(dataMap) {
            page.tableHeaders = dataMap.headers;
            page.tableRows = dataMap.rows;

            // 动态计算该表格的总宽度：(时间列的 180 + 其他列的数量 * 120)
            page.tableContentWidth = page.tableHeaders.length > 0 ? (180 + (page.tableHeaders.length - 1) * 120) : 0;

            if (page.tableRows.length === 0) {
                messagePopup.isError = false;
                messagePopup.message = "查询成功，但在设定的时间范围内没有数据。";
                messagePopup.open();
            }
        }

        function onTableQueryError(msg) {
            messagePopup.isError = true;
            messagePopup.message = "查询失败: " + msg;
            messagePopup.open();
        }
    }

    MessagePopup { id: messagePopup }
}
