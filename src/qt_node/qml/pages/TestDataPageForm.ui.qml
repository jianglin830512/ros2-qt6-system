import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0
import "../components"

Item {
    id: root

    // Aliases
    property alias dateInput: dateInput
    property alias timeInput: timeInput
    property alias spanCombo: spanCombo
    property alias circuitCombo: circuitCombo
    property alias queryBtn: queryBtn
    property alias tableListView: tableListView
    property alias headerRowLayout: headerRowLayout

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        // === 顶部：查询控制栏 ===
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
                spacing: 20

                // 日期
                RowLayout {
                    Label { text: "日期:"; color: Theme.textColor; font: Theme.defaultFont }
                    Rectangle {
                        width: 140; height: 36; color: Theme.highlightColor; radius: 5
                        TextInput {
                            id: dateInput
                            anchors.fill: parent; anchors.leftMargin: 5
                            verticalAlignment: Text.AlignVCenter
                            color: "white"; font: Theme.defaultFont
                            text: Qt.formatDateTime(new Date(), "yyyy-MM-dd")
                            inputMask: "9999-99-99"
                        }
                    }
                }

                // 时间
                RowLayout {
                    Label { text: "时间:"; color: Theme.textColor; font: Theme.defaultFont }
                    Rectangle {
                        width: 80; height: 36; color: Theme.highlightColor; radius: 5
                        TextInput {
                            id: timeInput
                            anchors.fill: parent; anchors.leftMargin: 5
                            verticalAlignment: Text.AlignVCenter
                            color: "white"; font: Theme.defaultFont
                            text: Qt.formatDateTime(new Date(), "hh:mm")
                            inputMask: "99:99"
                        }
                    }
                }

                // 时长
                RowLayout {
                    Label { text: "时长:"; color: Theme.textColor; font: Theme.defaultFont }
                    ComboBox {
                        id: spanCombo
                        width: 100
                        model: [2, 4, 8, 12, 24]
                        currentIndex: 0
                    }
                    Label { text: "小时"; color: Theme.textColor; font: Theme.defaultFont }
                }

                // 回路选择
                RowLayout {
                    Label { text: "目标回路:"; color: Theme.textColor; font: Theme.defaultFont }
                    ComboBox {
                        id: circuitCombo
                        width: 140
                        model: ["回路 1", "回路 2"]
                        currentIndex: 0
                    }
                }

                Item { Layout.fillWidth: true } // 弹簧

                // 查询按钮
                StyledButton {
                    id: queryBtn
                    text: "查询"
                    implicitWidth: 120
                    implicitHeight: 40
                    background: Rectangle {
                        color: Theme.highlightColor
                        border.color: Theme.orange
                        border.width: 2
                        radius: 5
                    }
                }
            }
        }

        // === 底部：表格数据区域 ===
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: Theme.controlBgColor
            border.color: Theme.highlightColor
            border.width: 2
            radius: 5
            clip: true // 保证边角裁切

            // 外层 Flickable 专门控制【横向滚动】，同步带动表头和表格
            Flickable {
                id: horizontalFlick
                anchors.fill: parent
                anchors.margins: 2
                // 固定总宽度：1列180 + 28列100 = 2980
                contentWidth: 2980
                // 纵向不在此处滚动，因此高度设为与容器相同
                contentHeight: height
                clip: true

                // 外层的横向滚动条
                ScrollBar.horizontal: ScrollBar {
                    policy: ScrollBar.AlwaysOn
                }

                Column {
                    anchors.fill: parent

                    // 1. 固定表头
                    Rectangle {
                        id: headerRect
                        width: 2980
                        height: 40
                        color: Theme.highlightColor

                        Row {
                            id: headerRowLayout
                            anchors.fill: parent
                        }
                    }

                    // 2. 数据列表 (占据剩余高度)
                    ListView {
                        id: tableListView
                        width: 2980
                        height: parent.height - headerRect.height
                        clip: true

                        // 纵向滚动条属于 ListView，仅数据区上下滚动
                        ScrollBar.vertical: ScrollBar {
                            policy: ScrollBar.AlwaysOn
                        }
                    }
                }
            }
        }
    }
}
