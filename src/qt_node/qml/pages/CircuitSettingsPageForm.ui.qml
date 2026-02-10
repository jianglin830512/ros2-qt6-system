import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0
import "../components"

Item {
    id: root

    // 【新增】传入回路ID (1 或 2)
    property int circuitId: 1

    // --- Aliases (保持不变) ---
    property alias testStartStop: testStartStop
    property alias testStartCurrent: testStartCurrent
    property alias testMaxCurrent: testMaxCurrent
    property alias testChangePercent: testChangePercent
    property alias testCtRatio: testCtRatio
    property alias testStartDate: testStartDate
    property alias testCycleCount: testCycleCount
    property alias testHeatFeedback: testHeatFeedback
    property alias testHeatInputHour: testHeatInputHour
    property alias testHeatInputMin: testHeatInputMin
    property alias testHeatingDuration: testHeatingDuration

    property alias refStartStop: refStartStop
    property alias refStartCurrent: refStartCurrent
    property alias refMaxCurrent: refMaxCurrent
    property alias refChangePercent: refChangePercent
    property alias refCtRatio: refCtRatio
    property alias refStartDate: refStartDate
    property alias refCycleCount: refCycleCount
    property alias refHeatFeedback: refHeatFeedback
    property alias refHeatInputHour: refHeatInputHour
    property alias refHeatInputMin: refHeatInputMin
    property alias refHeatingDuration: refHeatingDuration

    property alias refSourceDisplay: refSourceDisplay
    property alias refSourceCombo: refSourceCombo

    property alias sampleType: sampleType
    property alias sampleSpec: sampleSpec
    property alias sampleInsMaterial: sampleInsMaterial
    property alias sampleInsThick: sampleInsThick

    property alias applyLeftBtn: groupLeft.applyButton
    property alias applyMidBtn: groupMid.applyButton
    property alias applyRightBtn: groupRight.applyButton
    // 暴露还原按钮的别名
    property alias restoreLeftBtn: groupLeft.restoreButton
    property alias restoreMidBtn: groupMid.restoreButton
    property alias restoreRightBtn: groupRight.restoreButton

    property alias messagePopup: messagePopup

    RowLayout {
        anchors.fill: parent
        spacing: 20

        // =================================================
        // 左侧：试验回路
        // =================================================
        SettingsGroup {
            id: groupLeft
            // 【修改】动态标题
            title: "试验回路" + root.circuitId
            Layout.fillWidth: true
            Layout.fillHeight: true

            StartStopPanel {
                id: testStartStop
                Layout.fillWidth: true
                // 简单的ID计算: C1->1, C2->3
                loopId: (root.circuitId - 1) * 2 + 1
            }

            Label { text: "电流设置"; color: Theme.orange; font: Theme.defaultFont; Layout.alignment: Qt.AlignHCenter }
            ColumnLayout {
                Layout.fillWidth: true
                spacing: 10
                SettingInput { id: testStartCurrent; labelText: "起始电流:"; unitText: "A" }
                SettingInput { id: testMaxCurrent; labelText: "最大电流:"; unitText: "A" }
                SettingInput { id: testChangePercent; labelText: "电流变化:"; unitText: "%" }
                SettingInput { id: testCtRatio; labelText: "互感器变比:"; unitText: "/5A" }
            }

            Label { text: "时间设置"; color: Theme.orange; font: Theme.defaultFont; Layout.alignment: Qt.AlignHCenter; Layout.topMargin: 10 }
            ColumnLayout {
                Layout.fillWidth: true
                spacing: 10
                SettingInput {
                    id: testStartDate
                    labelText: "起始日期:"
                    unitText: ""
                    // 使用正则验证器，允许常见日期字符
                    inputValidator: RegularExpressionValidator { regularExpression: /^[0-9\-\/\.]+$/ }
                }
                SettingInput { id: testCycleCount; labelText: "循环次数:"; unitText: "次" }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 10
                    Item { Layout.fillWidth: true }
                    Label {
                        text: "加热时刻:"
                        color: Theme.textColor
                        font: Theme.defaultFont
                        Layout.alignment: Qt.AlignVCenter
                        Layout.preferredWidth: 110
                        horizontalAlignment: Text.AlignRight
                    }
                    Rectangle {
                        Layout.preferredWidth: 120; Layout.preferredHeight: 40
                        color: Theme.highlightColor; radius: 15
                        Label {
                            id: testHeatFeedback
                            anchors.centerIn: parent
                            text: "00:00"
                            color: "white"
                            font: Theme.defaultFont
                        }
                    }
                    RowLayout {
                        Layout.preferredWidth: 120; Layout.preferredHeight: 40; spacing: 5
                        Rectangle {
                            Layout.fillWidth: true; implicitHeight: 40
                            color: "transparent"; border.color: Theme.highlightColor; border.width: 2; radius: 10
                            TextInput {
                                id: testHeatInputHour; anchors.fill: parent; anchors.margins: 2
                                verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter
                                color: Theme.textColor; font: Theme.defaultFont; text: "00"
                                validator: IntValidator{bottom:0; top:23}
                                onActiveFocusChanged: parent.border.color = activeFocus ? Theme.orange : Theme.highlightColor
                            }
                        }
                        Label { text: ":"; color: Theme.textColor; font: Theme.defaultFont }
                        Rectangle {
                            Layout.fillWidth: true; implicitHeight: 40
                            color: "transparent"; border.color: Theme.highlightColor; border.width: 2; radius: 10
                            TextInput {
                                id: testHeatInputMin; anchors.fill: parent; anchors.margins: 2
                                verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter
                                color: Theme.textColor; font: Theme.defaultFont; text: "00"
                                validator: IntValidator{bottom:0; top:59}
                                onActiveFocusChanged: parent.border.color = activeFocus ? Theme.orange : Theme.highlightColor
                            }
                        }
                    }
                    Item { Layout.preferredWidth: 40 }
                    Item { Layout.fillWidth: true }
                }

                SettingInput { id: testHeatingDuration; labelText: "加热时长:"; unitText: "Min" }
            }
        }

        // =================================================
        // 中间：模拟回路
        // =================================================
        SettingsGroup {
            id: groupMid
            // 【修改】动态标题
            title: "模拟回路" + root.circuitId
            Layout.fillWidth: true
            Layout.fillHeight: true

            StartStopPanel {
                id: refStartStop
                Layout.fillWidth: true
                // 简单的ID计算: C1->2, C2->4
                loopId: (root.circuitId - 1) * 2 + 2
            }

            Label { text: "电流设置"; color: Theme.orange; font: Theme.defaultFont; Layout.alignment: Qt.AlignHCenter }
            ColumnLayout {
                Layout.fillWidth: true
                spacing: 10
                SettingInput { id: refStartCurrent; labelText: "起始电流:"; unitText: "A" }
                SettingInput { id: refMaxCurrent; labelText: "最大电流:"; unitText: "A" }
                SettingInput { id: refChangePercent; labelText: "电流变化:"; unitText: "%" }
                SettingInput { id: refCtRatio; labelText: "互感器变比:"; unitText: "/5A" }
            }

            Label { text: "时间设置"; color: Theme.orange; font: Theme.defaultFont; Layout.alignment: Qt.AlignHCenter; Layout.topMargin: 10 }
            ColumnLayout {
                Layout.fillWidth: true
                spacing: 10
                SettingInput {
                    id: refStartDate
                    labelText: "起始日期:"
                    unitText: ""
                    inputValidator: RegularExpressionValidator { regularExpression: /^[0-9\-\/\.]+$/ }
                }
                SettingInput { id: refCycleCount; labelText: "循环次数:"; unitText: "次" }

                RowLayout {
                    Layout.fillWidth: true
                    spacing: 10
                    Item { Layout.fillWidth: true }
                    Label {
                        text: "加热时刻:"
                        color: Theme.textColor
                        font: Theme.defaultFont
                        Layout.alignment: Qt.AlignVCenter
                        Layout.preferredWidth: 110
                        horizontalAlignment: Text.AlignRight
                    }
                    Rectangle {
                        Layout.preferredWidth: 120; Layout.preferredHeight: 40
                        color: Theme.highlightColor; radius: 15
                        Label { id: refHeatFeedback; anchors.centerIn: parent; text: "00:00"; color: "white"; font: Theme.defaultFont }
                    }
                    RowLayout {
                        Layout.preferredWidth: 120; Layout.preferredHeight: 40; spacing: 5
                        Rectangle {
                            Layout.fillWidth: true; implicitHeight: 40
                            color: "transparent"; border.color: Theme.highlightColor; border.width: 2; radius: 10
                            TextInput {
                                id: refHeatInputHour; anchors.fill: parent; anchors.margins: 2
                                verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter
                                color: Theme.textColor; font: Theme.defaultFont; text: "00"
                                validator: IntValidator{bottom:0; top:23}
                                onActiveFocusChanged: parent.border.color = activeFocus ? Theme.orange : Theme.highlightColor
                            }
                        }
                        Label { text: ":"; color: Theme.textColor; font: Theme.defaultFont }
                        Rectangle {
                            Layout.fillWidth: true; implicitHeight: 40
                            color: "transparent"; border.color: Theme.highlightColor; border.width: 2; radius: 10
                            TextInput {
                                id: refHeatInputMin; anchors.fill: parent; anchors.margins: 2
                                verticalAlignment: Text.AlignVCenter; horizontalAlignment: Text.AlignHCenter
                                color: Theme.textColor; font: Theme.defaultFont; text: "00"
                                validator: IntValidator{bottom:0; top:59}
                                onActiveFocusChanged: parent.border.color = activeFocus ? Theme.orange : Theme.highlightColor
                            }
                        }
                    }
                    Item { Layout.preferredWidth: 40 }
                    Item { Layout.fillWidth: true }
                }

                SettingInput { id: refHeatingDuration; labelText: "加热时长:"; unitText: "Min" }
            }
        }

        // =================================================
        // 右侧
        // =================================================
        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            spacing: 20

            // --- 右上：回路设定 ---
            SettingsGroup {
                id: groupRightTop
                // 【修改】动态标题
                title: "回路" + root.circuitId + "设定"
                Layout.fillWidth: true
                Layout.fillHeight: false
                Layout.preferredHeight: 200
                applyButton.visible: false
                restoreButton.visible: false

                Item { Layout.fillHeight: true }
                RowLayout {
                    Layout.fillWidth: true
                    spacing: 15
                    Item { Layout.fillWidth: true }
                    Label {
                        text: "恒流参考值:"
                        color: Theme.orange;
                        font: Theme.defaultFont
                        Layout.alignment: Qt.AlignVCenter
                    }
                    Rectangle {
                        Layout.preferredWidth: 140
                        Layout.preferredHeight: 40
                        color: Theme.highlightColor
                        radius: 5
                        Layout.alignment: Qt.AlignVCenter
                        Label {
                            id: refSourceDisplay
                            anchors.centerIn: parent
                            text: "读取中..."
                            color: "white"
                            font: Theme.defaultFont
                        }
                    }
                    ComboBox {
                        id: refSourceCombo
                        Layout.preferredWidth: 140
                        Layout.preferredHeight: 40
                        Layout.alignment: Qt.AlignVCenter
                        font: Theme.defaultFont
                        // 【修改】动态下拉内容
                        model: ["试验回路" + root.circuitId, "模拟回路" + root.circuitId]
                    }
                    Item { Layout.fillWidth: true }
                }
                Item { Layout.fillHeight: true }
            }

            // --- 右下：被试品 ---
            SettingsGroup {
                id: groupRight
                title: "被试品"
                Layout.fillWidth: true
                Layout.fillHeight: true

                ColumnLayout {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    spacing: 15
                    SettingTextArea { id: sampleType; title: "电缆类型"; Layout.fillWidth: true; inputHeight: 80 }
                    SettingTextArea { id: sampleSpec; title: "电缆规格"; Layout.fillWidth: true; inputHeight: 80 }
                    SettingTextArea { id: sampleInsMaterial; title: "绝缘类型"; Layout.fillWidth: true; inputHeight: 80 }
                    SettingInput { id: sampleInsThick; labelText: "绝缘厚度: "; unitText: "mm"; Layout.fillWidth: true
                        // 添加正则验证器：
                        // ^\d* 表示匹配开头任意个数字
                        // (\.\d{0,2})? 表示匹配可选的小数点，且小数点后最多跟2位数字
                        inputValidator: RegularExpressionValidator {
                            regularExpression: /^\d*(\.\d{0,2})?$/
                        }
                    }
                }
            }
        }
    }
    MessagePopup { id: messagePopup }
}
