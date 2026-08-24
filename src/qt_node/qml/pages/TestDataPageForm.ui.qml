import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0
import "../components"

Item {
    id: root

    // Aliases
    property alias queryPanel: queryPanel
    property alias tableListView: tableListView
    property alias headerRowLayout: headerRowLayout

    // 暴露根属性：动态计算的内容总宽度，通过 TestDataPage.qml 自动更新
    property int tableContentWidth: 0

    ColumnLayout {
        anchors.fill: parent
        anchors.margins: 10
        spacing: 10

        // === 顶部：通用查询控制栏 ===
        QueryPanel {
            id: queryPanel
        }

        // === 底部：表格数据区域 ===
        Rectangle {
            Layout.fillWidth: true
            Layout.fillHeight: true
            color: Theme.controlBgColor
            border.color: Theme.highlightColor
            border.width: 2
            radius: 5
            clip: true

            // 【核心修复】：增加一个带有 2px margins 的容器，保护外层边框不被列表背景覆盖
            Item {
                anchors.fill: parent
                anchors.margins: 2

                // 表头部分：只作为背景展示，不截获手势，跟随下方的 ListView 联动
                Flickable {
                    id: headerFlick
                    anchors.top: parent.top
                    anchors.left: parent.left
                    anchors.right: parent.right
                    height: 60 // 为了容纳换行，表头高度提升为 60
                    contentWidth: root.tableContentWidth
                    interactive: false
                    clip: true

                    Rectangle {
                        width: root.tableContentWidth
                        height: 60
                        color: Theme.highlightColor // 深蓝色底色

                        Row {
                            id: headerRowLayout
                            anchors.fill: parent
                        }
                    }
                }

                // 数据列表：利用 ListView 原生的内容滚动实现二维拖动
                ListView {
                    id: tableListView
                    anchors.top: headerFlick.bottom
                    anchors.left: parent.left
                    anchors.right: parent.right
                    anchors.bottom: parent.bottom

                    contentWidth: root.tableContentWidth
                    clip: true

                    // 表格横向拖动时，同步更新顶部的表头坐标
                    onContentXChanged: {
                        headerFlick.contentX = contentX;
                    }

                    // 完美的水平及垂直滚动条，垂直滑块将永远出现在屏幕最右侧边缘
                    ScrollBar.horizontal: ScrollBar {
                        policy: ScrollBar.AlwaysOn
                    }
                    ScrollBar.vertical: ScrollBar {
                        policy: ScrollBar.AlwaysOn
                    }
                }
            }
        }
    }
}
