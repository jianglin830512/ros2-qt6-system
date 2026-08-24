import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

Rectangle {
    id: root
    implicitWidth: 350
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

    // 新增：分离后的左/右温度循环生成器 以及标题参数
    property alias leftTempRepeater: leftTempRepeater
    property alias rightTempRepeater: rightTempRepeater
    property string leftCardTitle: ""
    property string rightCardTitle: ""

    property alias closeBreakerButton: closeBreakerButton
    property alias openBreakerButton: openBreakerButton

    // 【修改点】：全局遮罩别名绑定至容器，不再覆盖底部温度
    property alias isBlocked: blockerContainer.visible
    property bool isButtonsBlocked: false

    ColumnLayout {
        id: mainLayout
        anchors.fill: parent; anchors.margins: 10; spacing: 8

        Item {
            id: titleArea
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 50
            Layout.minimumHeight: 40
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

        Rectangle {
            id: currentValuePanle
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 120
            Layout.minimumHeight: 105
            color: "transparent"
            border.color: Theme.circuitCurrentValue
            border.width: 3
            radius: 8

            Row {
                anchors.centerIn: parent
                spacing: 15
                Label {
                    id: measureCurrentLabel
                    text: "0"
                    color: Theme.circuitCurrentValue
                    font.pixelSize: 100
                    font.bold: true
                    font.family: "Courier New"
                    anchors.verticalCenter: parent.verticalCenter
                }
                Label {
                    text: "A"
                    color: Theme.circuitCurrentValue
                    font.pixelSize: 80
                    font.bold: true
                    font.family: "Arial"
                    anchors.bottom: measureCurrentLabel.bottom
                    anchors.bottomMargin: 14
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
                    enabled: !root.isButtonsBlocked
                }
                ToggleActionButton {
                    id: openBreakerButton
                    labelText: "分闸"
                    colorWhenOn: "lime"
                    enabled: !root.isButtonsBlocked
                }
            }
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
                    Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 100
                    title: "加热设定:"; value: heatSetValue; unit: "min"
                }
                Item { Layout.preferredWidth: 20 }
                ValueAndUnit{
                    Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 70
                    title: "剩余:"; value: heatRemainValue; unit: "min"
                }
                ValueAndUnit{
                    Layout.fillWidth: true; Layout.fillHeight: true
                    title: "循环设定:"; value: cycleSetValue; unit: "次"
                }
                Item { Layout.preferredWidth: 20 }
                ValueAndUnit{
                    Layout.fillWidth: true; Layout.fillHeight: true
                    title: "剩余:"; value: cycleRemainValue; unit: "次"
                }
            }
        }

        Label {
            id: tempTitleLabel
            text: "温度"
            color: Theme.orange
            font: Theme.defaultFont
            Layout.alignment: Qt.AlignHCenter
        }

        Item {
            id: temperaturePanle
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredHeight: 500

            RowLayout {
                anchors.fill: parent
                spacing: 15

                // 1. 左侧栏 (通道 1~8)
                ColumnLayout {
                    Layout.fillWidth: true
                    // 【核心修改】给左右各分配权重 1，强制 50/50 分配
                    Layout.preferredWidth: 1
                    Layout.fillHeight: true

                    Label {
                        text: root.leftCardTitle
                        color: Theme.titleColor
                        font: Theme.smallLabelFont
                        visible: root.leftCardTitle !== ""
                        Layout.alignment: Qt.AlignHCenter
                    }
                    ColumnLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: 2
                        Repeater {
                            id: leftTempRepeater
                            delegate: ValueAndUnit {
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                title: modelData.titleName
                                value: modelData.value
                                unit: "℃"
                            }
                        }
                    }
                }

                // 2. 右侧栏 (通道 9~16 / 空白预留区)
                ColumnLayout {
                    Layout.fillWidth: true
                    // 【核心修改】这里也分配权重 1。当没有数据时，它依然霸占着 50% 宽度，完美预留空白
                    Layout.preferredWidth: 1
                    Layout.fillHeight: true

                    Label {
                        text: root.rightCardTitle
                        color: Theme.titleColor
                        font: Theme.smallLabelFont
                        visible: root.rightCardTitle !== ""
                        Layout.alignment: Qt.AlignHCenter
                    }
                    ColumnLayout {
                        Layout.fillWidth: true
                        Layout.fillHeight: true
                        spacing: 2
                        Repeater {
                            id: rightTempRepeater
                            delegate: ValueAndUnit {
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                title: modelData.titleName
                                value: modelData.value
                                unit: "℃"
                            }
                        }
                    }
                }
            }
        }
    }

    // 面板局部遮罩容器：高度经过精算，完美止步于 tempTitleLabel 的上方边界，使得遮罩不再覆盖温度显示区
    Item {
        id: blockerContainer
        x: 0
        y: titleArea.height + 15
        width: root.width
        // 高度计算：(温度Label在布局中绝对高度 - 容器的起始点高度)，由于 mainLayout 有 margins: 10，所以补充 +10 的偏移
        height: Math.max(0, tempTitleLabel.y + 10 - y)
        z: 999
        visible: false // 实际受 isBlocked 别名控制

        InputBlocker {
            anchors.fill: parent
            radius: root.radius
            statusText: ""
            overlayColor: Theme.blockerOverlayColor
            visible: true // 内部设为真，交由外部容器控制其显示
        }
    }
}
