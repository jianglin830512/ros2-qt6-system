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
    property alias sortCombo: sortCombo
    property alias orderCombo: orderCombo

    property alias tableListView: tableListView
    property alias headerRowLayout: headerRowLayout

    property alias btnPrevPage: btnPrevPage
    property alias btnNextPage: btnNextPage
    property alias pageLabel: pageLabel

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
                anchors.fill: parent
                anchors.margins: 10
                spacing: 15

                Label { text: "名称关键字:"; color: Theme.textColor; font: Theme.defaultFont }

                Rectangle {
                    Layout.preferredWidth: 160; Layout.preferredHeight: 38
                    color: "transparent"; border.color: Theme.buttonBorderColor; border.width: 2; radius: 5
                    TextInput {
                        id: searchInput
                        anchors.fill: parent; anchors.margins: 5
                        verticalAlignment: Text.AlignVCenter; color: Theme.textColor; font: Theme.defaultFont
                        clip: true
                    }
                }

                StyledButton { id: btnSearch; text: "搜 索"; implicitWidth: 80; implicitHeight: 38 }

                Label { text: "排序字段:"; color: Theme.textColor; font: Theme.defaultFont; Layout.leftMargin: 15 }
                ComboBox {
                    id: sortCombo
                    Layout.preferredWidth: 140; Layout.preferredHeight: 38
                    // 【修改点】：将“最后修改时间”改为“修改时间”
                    model: ["修改时间", "名称", "电芯直径", "绝缘厚度", "电压等级", "系统制式"]
                    font: Theme.defaultFont
                }

                Label { text: "顺序:"; color: Theme.textColor; font: Theme.defaultFont; Layout.leftMargin: 5 }
                ComboBox {
                    id: orderCombo
                    Layout.preferredWidth: 90; Layout.preferredHeight: 38
                    model: ["倒序", "正序"]
                    font: Theme.defaultFont
                }

                Item { Layout.fillWidth: true } // 弹簧占位

                StyledButton { id: btnAdd; text: "+ 新增电缆"; implicitWidth: 140; implicitHeight: 40 }
            }
        }

        // === 中间：表格数据区域 ===
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: Theme.controlBgColor
            border.color: Theme.highlightColor
            border.width: 2
            radius: 5
            clip: true

            Item {
                anchors.fill: parent
                anchors.margins: 2

                Flickable {
                    id: headerFlick
                    anchors.top: parent.top; anchors.left: parent.left; anchors.right: parent.right
                    height: 50; contentWidth: root.tableContentWidth; interactive: false; clip: true

                    Rectangle {
                        width: root.tableContentWidth; height: 50; color: Theme.highlightColor
                        Row { id: headerRowLayout; anchors.fill: parent }
                    }
                }

                ListView {
                    id: tableListView
                    anchors.top: headerFlick.bottom; anchors.left: parent.left; anchors.right: parent.right; anchors.bottom: parent.bottom
                    contentWidth: root.tableContentWidth
                    clip: true
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
                anchors.centerIn: parent
                spacing: 30

                StyledButton { id: btnPrevPage; text: "上一页"; implicitWidth: 100; implicitHeight: 40 }
                Label { id: pageLabel; text: "1 / 1"; color: Theme.textColor; font: Theme.largeLabelFont }
                StyledButton { id: btnNextPage; text: "下一页"; implicitWidth: 100; implicitHeight: 40 }
            }
        }
    }
}
