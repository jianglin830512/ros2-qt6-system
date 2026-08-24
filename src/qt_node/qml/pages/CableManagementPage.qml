import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt_node 1.0
import qt.theme 1.0
import "../components"

CableManagementPageForm {
    id: page

    property int currentPage: 1
    property int totalPages: 1
    property int pageSize: 20

    // 废弃固定像素 width，改为比例 weight
    property var tableHeaders: [
        {title: "名称", weight: 1.5},
        {title: "电芯直径(mm)", weight: 1.0},
        {title: "电芯材料", weight: 1.0},
        {title: "绝缘厚度(mm)", weight: 1.0},
        {title: "绝缘材料", weight: 1.0},
        {title: "电压等级(kV)", weight: 1.0}, // 【修改点】改为 kV
        {title: "系统制式", weight: 0.8},
        {title: "备注", weight: 1.5},
        {title: "修改时间", weight: 1.5}, // 【修改点】改为 修改时间
        {title: "操作", weight: 1.0}
    ]

    property real totalWeight: 11.3 // 上述权重总和
    property var tableRows: []

    // 利用属性绑定，使表格宽度自动平铺占满；如果窗口被极度压缩，设定 1400 为底线，从而触发滚动条
    tableContentWidth: Math.max(1400, tableListView.width)

    // 行高绑定 ListView 的实际高度 / 20，让 20 行完全填满纵向，极度压缩时保留 30 的最小行高触发滚动条
    property real dynamicRowHeight: Math.max(30, tableListView.height / page.pageSize)

    onVisibleChanged: {
        if (visible) {
            loadData();
        }
    }

    function loadData() {
        let isAscending = page.orderCombo.currentIndex === 1;
        rosProxy.listCables(page.searchInput.text, page.currentPage, page.pageSize, page.sortCombo.currentIndex, isAscending);
    }

    // 1. 生成表头 (动态分配宽度)
    Repeater {
        parent: page.headerRowLayout
        model: page.tableHeaders
        delegate: Rectangle {
            // 宽度按权重占屏幕动态宽度的比例
            width: (modelData.weight / page.totalWeight) * page.tableContentWidth
            height: 50
            color: "transparent"; border.color: Theme.gridLineColor; border.width: 1
            Text {
                anchors.centerIn: parent; text: modelData.title
                color: Theme.buttonSelectedTextColor; font: Theme.defaultFont
            }
        }
    }

    // 2. 数据行视图委托
    tableListView.model: page.tableRows
    tableListView.delegate: Item {
        id: rowItem
        width: page.tableContentWidth
        height: page.dynamicRowHeight // 行高应用动态计算值
        property var rowData: modelData

        Rectangle {
            anchors.fill: parent
            color: ma.containsMouse ? Theme.tableHoverColor : (index % 2 === 0 ? "transparent" : Theme.tableAlternateColor)
        }

        MouseArea { id: ma; anchors.fill: parent; hoverEnabled: true }

        Row {
            height: parent.height
            // 按照列顺序依次提取数据，同样按权重动态分配宽度
            Rectangle { width: (page.tableHeaders[0].weight / page.totalWeight) * page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: rowData.name; color: Theme.valueColor; font: Theme.smallLabelFont; elide: Text.ElideRight; width: parent.width-10; horizontalAlignment: Text.AlignHCenter } }
            Rectangle { width: (page.tableHeaders[1].weight / page.totalWeight) * page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: Number(rowData.core_diameter).toFixed(2); color: Theme.valueColor; font: Theme.smallLabelFont } }
            Rectangle { width: (page.tableHeaders[2].weight / page.totalWeight) * page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: rowData.core_material; color: Theme.valueColor; font: Theme.smallLabelFont; elide: Text.ElideRight; width: parent.width-10; horizontalAlignment: Text.AlignHCenter } }
            Rectangle { width: (page.tableHeaders[3].weight / page.totalWeight) * page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: Number(rowData.insulation_thickness).toFixed(2); color: Theme.valueColor; font: Theme.smallLabelFont } }
            Rectangle { width: (page.tableHeaders[4].weight / page.totalWeight) * page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: rowData.insulation_material; color: Theme.valueColor; font: Theme.smallLabelFont; elide: Text.ElideRight; width: parent.width-10; horizontalAlignment: Text.AlignHCenter } }

            // 【修改点】电压等级展示时除以 1000 转换为 kV 并取整
            Rectangle { width: (page.tableHeaders[5].weight / page.totalWeight) * page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: Math.round(rowData.voltage_grade / 1000); color: Theme.valueColor; font: Theme.smallLabelFont } }
            Rectangle { width: (page.tableHeaders[6].weight / page.totalWeight) * page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: rowData.system_format === 0 ? "直流" : "交流"; color: Theme.valueColor; font: Theme.smallLabelFont } }

            Rectangle { width: (page.tableHeaders[7].weight / page.totalWeight) * page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: rowData.notes; color: Theme.valueColor; font: Theme.smallLabelFont; elide: Text.ElideRight; width: parent.width-10; horizontalAlignment: Text.AlignHCenter } }
            Rectangle { width: (page.tableHeaders[8].weight / page.totalWeight) * page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1
                Text { anchors.centerIn: parent; text: rowData.last_modified; color: Theme.valueColor; font: Theme.smallLabelFont } }

            // 操作列
            Rectangle {
                width: (page.tableHeaders[9].weight / page.totalWeight) * page.tableContentWidth; height: parent.height; color: "transparent"; border.color: Theme.gridLineTransparentColor; border.width: 1

                // 将 RowLayout 改为普通的 Row，避免布局引擎高度塌陷
                Row {
                    anchors.centerIn: parent
                    spacing: 10

                    // 修改按钮
                    Rectangle {
                        width: 50; height: 28; // 恢复固定高度 28
                        color: editMa.pressed ? Theme.buttonSelectedGradientStart : Theme.highlightColor; radius: 4
                        Text { anchors.centerIn: parent; text: "修改"; color: "white"; font: Theme.smallLabelFont }
                        MouseArea { id: editMa; anchors.fill: parent; onClicked: {
                                editDialog.cableId = rowData.id;
                                editDialog.nameText = rowData.name;
                                editDialog.coreDiaText = rowData.core_diameter.toString();
                                editDialog.coreMatText = rowData.core_material;
                                editDialog.insThickText = rowData.insulation_thickness.toString();
                                editDialog.insMatText = rowData.insulation_material;
                                // 【修改点】填入弹窗前除以 1000
                                editDialog.voltageGradeText = Math.round(rowData.voltage_grade / 1000).toString();
                                editDialog.systemFormatIndex = rowData.system_format;
                                editDialog.notesText = rowData.notes;
                                editDialog.open();
                            }}
                    }

                    // 删除按钮
                    Rectangle {
                        width: 50; height: 28; // 恢复固定高度 28
                        color: delMa.pressed ? Theme.buttonDangerPressedColor : Theme.buttonDangerColor; radius: 4
                        Text { anchors.centerIn: parent; text: "删除"; color: "white"; font: Theme.smallLabelFont }
                        MouseArea { id: delMa; anchors.fill: parent; onClicked: {
                                deleteConfirmDialog.targetId = rowData.id;
                                deleteConfirmDialog.targetName = rowData.name;
                                deleteConfirmDialog.open();
                            }}
                    }
                }
            }
        }
    }

    // 3. UI 交互控制
    btnSearch.onClicked: { page.currentPage = 1; loadData(); }
    sortCombo.onActivated: { page.currentPage = 1; loadData(); }
    orderCombo.onActivated: { page.currentPage = 1; loadData(); }

    btnPrevPage.onClicked: {
        if (page.currentPage > 1) { page.currentPage--; loadData(); }
    }
    btnNextPage.onClicked: {
        if (page.currentPage < page.totalPages) { page.currentPage++; loadData(); }
    }

    btnAdd.onClicked: {
        editDialog.cableId = -1;
        editDialog.nameText = "";
        editDialog.coreDiaText = "";
        editDialog.coreMatText = "";
        editDialog.insThickText = "";
        editDialog.insMatText = "";
        editDialog.voltageGradeText = "0";
        editDialog.systemFormatIndex = 0;
        editDialog.notesText = "";
        editDialog.open();
    }

    // 4. 后端连接
    Connections {
        target: rosProxy

        function onCablesListed(result) {
            if (result.success) {
                page.totalPages = result.total_pages;
                page.currentPage = result.current_page;
                page.pageLabel.text = result.current_page + " / " + result.total_pages;
                page.tableRows = result.cables;
            } else {
                messagePopup.isError = true;
                messagePopup.message = "查询失败: " + result.message;
                messagePopup.open();
            }
        }

        function onCableSaveResult(success, message) {
            if (success) {
                loadData(); // 保存成功后刷新
            } else {
                messagePopup.isError = true;
                messagePopup.message = "保存失败: " + message;
                messagePopup.open();
            }
        }

        function onCableDeleteResult(success, message) {
            if (success) {
                // 如果当前页被删光了，往前翻一页
                if (page.tableRows.length === 1 && page.currentPage > 1) {
                    page.currentPage--;
                }
                loadData();
            } else {
                messagePopup.isError = true;
                messagePopup.message = "删除失败: " + message;
                messagePopup.open();
            }
        }
    }

    // 5. 弹窗组件
    CableEditDialog {
        id: editDialog
        onSaveRequested: (id, name, coreDia, coreMat, insThick, insMat, voltageGrade, systemFormat, notes) => {
                             var cableMap = {
                                 "id": id,
                                 "name": name,
                                 "core_diameter": coreDia,
                                 "core_material": coreMat,
                                 "insulation_thickness": insThick,
                                 "insulation_material": insMat,
                                 "voltage_grade": voltageGrade * 1000, // 保存前将 kV 乘以 1000 转换为 V
                                 "system_format": systemFormat,
                                 "notes": notes
                             };
                             rosProxy.saveCable(cableMap);
                         }
    }

    Dialog {
        id: deleteConfirmDialog
        property int targetId: -1
        property string targetName: ""

        title: "确认删除"
        modal: true
        anchors.centerIn: Overlay.overlay
        width: 350
        background: Rectangle { color: Theme.controlBgColor; border.color: Theme.highlightColor; border.width: 2; radius: 8 }
        header: Label { text: title; color: Theme.errorColor; font: Theme.subjectFont; padding: 15; horizontalAlignment: Text.AlignHCenter }
        contentItem: Label {
            text: "确定要永久删除电缆 [" + deleteConfirmDialog.targetName + "] 吗？"
            color: Theme.textColor; font: Theme.defaultFont; wrapMode: Text.Wrap
            horizontalAlignment: Text.AlignHCenter; topPadding: 20; bottomPadding: 20
        }
        footer: DialogButtonBox {
            alignment: Qt.AlignHCenter
            background: Rectangle { color: "transparent" }
            padding: 15
            Button {
                text: "确定删除"
                implicitWidth: 100; implicitHeight: 40
                background: Rectangle { color: Theme.buttonDangerColor; radius: 5 }
                contentItem: Text { text: parent.text; color: "white"; font: Theme.buttonFont; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                onClicked: { rosProxy.deleteCable(deleteConfirmDialog.targetId); deleteConfirmDialog.close(); }
            }
            Button {
                text: "取消"
                implicitWidth: 100; implicitHeight: 40
                background: Rectangle { color: Theme.buttonHoverColor; radius: 5 }
                contentItem: Text { text: parent.text; color: Theme.textColor; font: Theme.buttonFont; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
                onClicked: deleteConfirmDialog.close()
            }
        }
    }

    MessagePopup { id: messagePopup }
}
