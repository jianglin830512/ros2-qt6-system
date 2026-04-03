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
        return index === 0 ? 180 : 100;
    }

    // ==========================================================
    // 智能日期与时间清洗函数
    // ==========================================================
    function normalizeDateStr(str) {
        if (!str) return "";
        var cleanStr = str.replace(/[\/\.]/g, "-");
        var parts = cleanStr.split("-");
        if (parts.length === 3) {
            var y = parts[0];
            var m = parts[1].toString().padStart(2, '0');
            var d = parts[2].toString().padStart(2, '0');
            return y + "-" + m + "-" + d;
        }
        return str;
    }

    function normalizeTimeStr(str) {
        if (!str) return "";
        // 允许用户不小心用点或者横杠代替冒号
        var cleanStr = str.replace(/[\/\.\-]/g, ":");
        var parts = cleanStr.split(":");
        if (parts.length === 2) {
            var h = parts[0].toString().padStart(2, '0');
            var m = parts[1].toString().padStart(2, '0');
            return h + ":" + m;
        }
        return str;
    }

    // ==========================================================
    // 监听 UI 层的输入完成事件 (回车或失去焦点)
    // ==========================================================
    Connections {
        target: page.dateInput
        function onEditingFinished() {
            page.dateInput.text = page.normalizeDateStr(page.dateInput.text);
        }
    }

    Connections {
        target: page.timeInput
        function onEditingFinished() {
            page.timeInput.text = page.normalizeTimeStr(page.timeInput.text);
        }
    }

    // 1. 动态生成表头
    Repeater {
        parent: page.headerRowLayout
        model: page.tableHeaders

        delegate: Rectangle {
            width: getColWidth(index)
            height: 40
            color: "transparent"
            border.color: Theme.gridLineColor
            border.width: 1

            Text {
                anchors.centerIn: parent
                text: modelData
                color: Theme.orange
                font: Theme.smallLabelFont
            }
        }
    }

    // 2. 数据行视图委托
    tableListView.model: page.tableRows
    tableListView.delegate: Item {
        id: rowItem
        // 宽度由内部的 Row 撑开，高度固定 35
        width: innerRow.width
        height: 35
        property var rowData: modelData // modelData 是 QVariantList 转化来的 JS Array

        // 鼠标悬停行高亮
        Rectangle {
            anchors.fill: parent
            color: ma.containsMouse ? Qt.rgba(Theme.highlightColor.r, Theme.highlightColor.g, Theme.highlightColor.b, 0.5) : (index % 2 === 0 ? "transparent" : "#1AFFFFFF")
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
                    border.color: Qt.rgba(Theme.gridLineColor.r, Theme.gridLineColor.g, Theme.gridLineColor.b, 0.3)
                    border.width: 1

                    Text {
                        anchors.centerIn: parent
                        text: modelData
                        color: "white"
                        font: Theme.smallLabelFont
                    }
                }
            }
        }
    }

    // 3. 按钮点击请求数据
    queryBtn.onClicked: {
        var dateStr = dateInput.text;
        var timeStr = timeInput.text;
        var span = parseInt(spanCombo.currentValue);
        var cid = circuitCombo.currentIndex + 1; // 0->1, 1->2

        rosProxy.queryTable(dateStr, timeStr, span, cid);
    }

    // 4. 处理后端发来的数据
    Connections {
        target: rosProxy

        function onTableDataReady(dataMap) {
            page.tableHeaders = dataMap.headers;
            page.tableRows = dataMap.rows;

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
