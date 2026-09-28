import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0
import "../components"
Item {
    id: root

    property alias searchInput: searchInput
    property alias btnSearch: btnSearch
    property alias btnAdd: btnAdd
    property alias btnExport: btnExport
    property alias tableListView: tableListView
    property alias headerRowLayout: headerRowLayout
    property alias btnPrevPage: btnPrevPage
    property alias btnNextPage: btnNextPage
    property alias pageLabel: pageLabel
    property alias detailsContainer: detailsContainer

    // 暴露 5 个信息 Label，供逻辑层填充选中的数据
    property alias infoCircuitId: infoCircuitId
    property alias infoStartDate: infoStartDate
    property alias infoEndDate: infoEndDate
    property alias infoCableName: infoCableName
    property alias infoRemarks: infoRemarks

    property real tableContentWidth: 1200

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        // === 顶部：控制栏 ===
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 70
            color: Theme.controlBgColor
            border.color: Theme.highlightColor
            border.width: 2
            radius: 5

            RowLayout {
                anchors.fill: parent; anchors.margins: 10; spacing: 15
                Label { text: "关键字:"; color: Theme.textColor; font: Theme.defaultFont }
                Rectangle {
                    Layout.preferredWidth: 160; Layout.preferredHeight: 38
                    color: "transparent"; border.color: Theme.buttonBorderColor; border.width: 2; radius: 5
                    TextInput { id: searchInput; anchors.fill: parent; anchors.margins: 5; verticalAlignment: Text.AlignVCenter; color: Theme.textColor; font: Theme.defaultFont }
                }
                StyledButton { id: btnSearch; text: "搜 索"; implicitWidth: 80; implicitHeight: 38 }
                Item { Layout.fillWidth: true }
                StyledButton { id: btnExport; text: "导出 CSV"; implicitWidth: 120; implicitHeight: 40 }
                StyledButton { id: btnAdd; text: "+ 新增记录"; implicitWidth: 140; implicitHeight: 40 }
            }
        }

        // === 中间：表格数据区域 ===
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 400
            color: Theme.controlBgColor
            border.color: Theme.highlightColor
            border.width: 2
            radius: 5
            clip: true

            Item {
                anchors.fill: parent; anchors.margins: 2
                Flickable {
                    id: headerFlick; anchors.top: parent.top; anchors.left: parent.left; anchors.right: parent.right
                    height: 50; contentWidth: root.tableContentWidth; interactive: false; clip: true
                    Rectangle { width: root.tableContentWidth; height: 50; color: Theme.highlightColor
                        Row { id: headerRowLayout; anchors.fill: parent } }
                }
                ListView {
                    id: tableListView; anchors.top: headerFlick.bottom; anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom
                    contentWidth: root.tableContentWidth; clip: true
                    onContentXChanged: { headerFlick.contentX = contentX; }
                    ScrollBar.horizontal: ScrollBar { policy: ScrollBar.AsNeeded }
                    ScrollBar.vertical: ScrollBar { policy: ScrollBar.AsNeeded }
                }
            }
        }

        // === 底部：分页控件 ===
        Rectangle {
            Layout.fillWidth: true
            Layout.preferredHeight: 50
            color: "transparent"

            RowLayout {
                anchors.centerIn: parent; spacing: 30
                StyledButton { id: btnPrevPage; text: "上一页"; implicitWidth: 100; implicitHeight: 40 }
                Label { id: pageLabel; text: "1 / 1"; color: Theme.textColor; font: Theme.largeLabelFont }
                StyledButton { id: btnNextPage; text: "下一页"; implicitWidth: 100; implicitHeight: 40 }
            }
        }

        // === 详情：测温点位置展示 ===
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 300
            color: Theme.controlBgColor
            border.color: Theme.highlightColor
            border.width: 2
            radius: 5

            ColumnLayout {
                anchors.fill: parent
                anchors.margins: 10
                spacing: 10

                // === 【修改点】固定的 2行5列 信息展示区 ===
                Rectangle {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 70
                    color: "transparent"
                    border.color: Theme.gridLineColor
                    border.width: 1
                    radius: 3

                    GridLayout {
                        anchors.fill: parent
                        anchors.margins: 5
                        columns: 5
                        rowSpacing: 5
                        columnSpacing: 5

                        // 表头 (第一行)
                        Rectangle { Layout.fillWidth: true; Layout.fillHeight: true; color: Theme.highlightColor; radius: 3; Label { anchors.centerIn: parent; text: "回路ID"; color: "white"; font: Theme.smallLabelFont } }
                        Rectangle { Layout.fillWidth: true; Layout.fillHeight: true; color: Theme.highlightColor; radius: 3; Label { anchors.centerIn: parent; text: "起始日期"; color: "white"; font: Theme.smallLabelFont } }
                        Rectangle { Layout.fillWidth: true; Layout.fillHeight: true; color: Theme.highlightColor; radius: 3; Label { anchors.centerIn: parent; text: "结束日期"; color: "white"; font: Theme.smallLabelFont } }
                        Rectangle { Layout.fillWidth: true; Layout.fillHeight: true; color: Theme.highlightColor; radius: 3; Label { anchors.centerIn: parent; text: "电缆名称"; color: "white"; font: Theme.smallLabelFont } }
                        Rectangle { Layout.fillWidth: true; Layout.fillHeight: true; color: Theme.highlightColor; radius: 3; Label { anchors.centerIn: parent; text: "备注"; color: "white"; font: Theme.smallLabelFont } }

                        // 数据 (第二行)
                        Rectangle { Layout.fillWidth: true; Layout.fillHeight: true; color: "transparent"; Label { id: infoCircuitId; anchors.centerIn: parent; text: ""; color: Theme.valueColor; font: Theme.smallLabelFont } }
                        Rectangle { Layout.fillWidth: true; Layout.fillHeight: true; color: "transparent"; Label { id: infoStartDate; anchors.centerIn: parent; text: ""; color: Theme.valueColor; font: Theme.smallLabelFont } }
                        Rectangle { Layout.fillWidth: true; Layout.fillHeight: true; color: "transparent"; Label { id: infoEndDate; anchors.centerIn: parent; text: ""; color: Theme.valueColor; font: Theme.smallLabelFont } }
                        Rectangle { Layout.fillWidth: true; Layout.fillHeight: true; color: "transparent"; Label { id: infoCableName; anchors.centerIn: parent; text: ""; color: Theme.valueColor; font: Theme.smallLabelFont; elide: Text.ElideRight; width: parent.width - 10; horizontalAlignment: Text.AlignHCenter } }
                        Rectangle { Layout.fillWidth: true; Layout.fillHeight: true; color: "transparent"; Label { id: infoRemarks; anchors.centerIn: parent; text: ""; color: Theme.valueColor; font: Theme.smallLabelFont; elide: Text.ElideRight; width: parent.width - 10; horizontalAlignment: Text.AlignHCenter } }
                    }
                }

                Rectangle {
                    id: detailsContainer
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                }
            }
        }
    }
}
