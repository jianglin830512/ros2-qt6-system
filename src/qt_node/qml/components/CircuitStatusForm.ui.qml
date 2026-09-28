import QtQuick
import QtQuick.Layouts
import qt.theme 1.0

Rectangle{
    color: "transparent"
    border.color: Theme.highlightColor
    border.width: 3
    radius: 8

    // --- Aliases for logic file ---
    property alias testLoop: testLoop
    property alias refLoop: refLoop
    property alias tempRepeater: tempRepeater

    ColumnLayout {
        anchors{
            fill: parent
            margins: 10
        }
        spacing: Theme.subSpacing

        // 上半部分：试验回路 & 模拟回路并排
        RowLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: Theme.subSpacing

            LoopStatus {
                id: testLoop
            }
            LoopStatus {
                id: refLoop
            }
        }

        // 下半部分：回路温度统一展示区
        Rectangle {
            Layout.fillWidth: true
            // 【修改点3】：原180增高到210，为第一行表头腾出空间
            Layout.preferredHeight: 210
            color: "transparent"
            border.color: Theme.highlightColor
            border.width: 2
            radius: 8

            GridLayout {
                anchors.fill: parent
                anchors.margins: 10
                columns: 9       // 1列表头 + 8列数据
                columnSpacing: 5
                rowSpacing: 5

                Repeater {
                    id: tempRepeater
                }
            }
        }
    }
}
