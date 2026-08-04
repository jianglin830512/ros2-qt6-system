import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

Rectangle {
    id: root
    implicitWidth: 350 // 给一个合理的最小宽度
    color: "transparent"
    border.color: Theme.highlightColor
    border.width: 1
    Layout.fillWidth: true
    Layout.fillHeight: true

    // --- Aliases for logic file ---
    property alias enableLabel: enableLabel
    property alias titleLabel: titleLabel
    property alias statusLabel: statusLabel
    property alias setCurrentLabel: setCurrentLabel
    property alias measureCurrentLabel: measureCurrentLabel
    property string heatSetValue: "0"
    property string heatRemainValue: "0"
    property string cycleSetValue: "0"
    property string cycleRemainValue: "0"
    property alias tempRepeater: tempRepeater
    property alias closeBreakerButton: closeBreakerButton
    property alias openBreakerButton: openBreakerButton

    // 【修改核心 1】：保留全局遮罩别名，将按钮遮罩变更为普通的 bool 属性
    property alias isBlocked: inputBlocker.visible
    property bool isButtonsBlocked: false

    ColumnLayout {
        anchors.fill: parent; anchors.margins: 10; spacing: 8

        Item {
            id: titleArea
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 50
            Layout.minimumHeight: 40 // 保护标题区域不被极限压缩
            RowLayout {
                anchors.fill: parent
                Item{
                    Layout.fillWidth: true
                    Text {
                        anchors.centerIn: parent
                        id: enableLabel
                        text: "停用"
                        font: Theme.defaultFont
                        color: "red"
                    }
                }
                Item{
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredHeight: 40
                    Text {
                        anchors.centerIn: parent
                        id: titleLabel
                        text: "回路"
                        font: Theme.subjectFont
                        color: Theme.orange
                    }
                }
                Item{
                    Layout.fillWidth: true
                    Text {
                        anchors.centerIn: parent
                        id: statusLabel
                        text: "状态"
                        font: Theme.defaultFont
                        color: "red"
                    }
                }
            }
        }

        Item {
            id: currentSettingPanle
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 50
            RowLayout{
                anchors.fill: parent
                Label {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    text: "设定电流:"
                    color: Theme.titleColor
                    font: Theme.largeLabelFont
                }
                Rectangle {
                    color: "transparent"
                    border.color: Theme.textColor
                    border.width: 2
                    radius: 5
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredWidth: 120
                    Label {
                        id: setCurrentLabel
                        text: "0"
                        color: Theme.valueColor
                        font: Theme.largeLabelFont
                        anchors.centerIn: parent
                    }
                }
                Label {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredWidth: 50
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                    text: "A"
                    color: Theme.titleColor
                    font: Theme.largeLabelFont
                }
            }
        }

        // 【纯净版】超大红色电流显示区 (整体绝对居中布局)
        Rectangle {
            id: currentValuePanle

            Layout.fillWidth: true
            Layout.fillHeight: true
            // 字体加到了100，高度再拉大一点确保容纳得下
            Layout.preferredHeight: 120
            Layout.minimumHeight: 105

            color: "transparent"
            border.color: Theme.circuitCurrentValue
            border.width: 3
            radius: 8

            // 放弃 Layout 弹簧，将数字和单位打包为一个普通的 Row，并在红框内绝对居中
            Row {
                anchors.centerIn: parent
                spacing: 15

                // 核心：红色超大数字
                Label {
                    id: measureCurrentLabel
                    text: "0"       // 初始化为占位符
                    color: Theme.circuitCurrentValue
                    font.pixelSize: 100  // 字体达到 100
                    font.bold: true
                    // 使用等宽字体，确保 0001.0 和 0008.0 占据完全相同的物理宽度，不抖动
                    font.family: "Courier New"
                    anchors.verticalCenter: parent.verticalCenter
                }

                // 单位 "A"
                Label {
                    text: "A"
                    color: Theme.circuitCurrentValue
                    font.pixelSize: 80   // 单位字体达到 80
                    font.bold: true
                    font.family: "Arial" // 单位不受宽度变化影响，保持清晰的 Arial
                    anchors.bottom: measureCurrentLabel.bottom
                    anchors.bottomMargin: 14 // 微调底部对齐，让A的底线跟数字持平
                }
            }
        }

        Item {
            id: breakerButtonPanle
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 60
            Layout.minimumHeight: 55

            RowLayout {
                anchors.centerIn: parent
                spacing: Theme.subSpacing
                ToggleActionButton {
                    id: closeBreakerButton
                    labelText: "合闸"
                    colorWhenOn: "red"
                    // 【修改核心 2】：直接绑定禁用状态
                    enabled: !root.isButtonsBlocked
                }
                ToggleActionButton {
                    id: openBreakerButton
                    labelText: "分闸"
                    colorWhenOn: "lime"
                    // 【修改核心 2】：直接绑定禁用状态
                    enabled: !root.isButtonsBlocked
                }
            }

            // 【修改核心 3】：彻底删除了原本在这里覆盖按钮的 buttonBlocker 遮罩
        }

        Item {
            id: timePanle
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 100
            GridLayout {
                anchors.fill: parent
                columns: 3
                rowSpacing: 5
                ValueAndUnit{
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredWidth: 100
                    title: "加热设定:"
                    value: heatSetValue
                    unit: "min"
                }
                Item {
                    Layout.preferredWidth: 20
                }
                ValueAndUnit{
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    Layout.preferredWidth: 70
                    title: "剩余:"
                    value: heatRemainValue
                    unit: "min"
                }
                ValueAndUnit{
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    title: "循环设定:"
                    value: cycleSetValue
                    unit: "次"
                }
                Item {
                    Layout.preferredWidth: 20
                }
                ValueAndUnit{
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    title: "剩余:"
                    value: cycleRemainValue
                    unit: "次"
                }
            }
        }

        Label { text: "温度"; color: Theme.orange; font: Theme.defaultFont;Layout.alignment: Layout.Center }

        Item {
            id: temperaturePanle
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 500
            GridLayout {
                anchors.fill: parent
                rows: 8
                flow: GridLayout.TopToBottom
                Repeater {
                    id: tempRepeater
                    delegate: ValueAndUnit {
                        Layout.preferredWidth: (parent.width - 30) / 2
                        Layout.fillHeight: true
                        title: modelData.titleName
                        value: modelData.value
                        unit: "℃"
                    }
                }
            }
        }
    }

    // 面板全局遮罩（保留，用于“回路停用”时的整体遮蔽）
    InputBlocker {
        id: inputBlocker
        radius: root.radius
        statusText: ""
        overlayColor: Theme.blockerOverlayColor
        anchors.topMargin: titleArea.height + 15
    }
}
