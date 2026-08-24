import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0
import "../components"

Item {
    id: root

    property int circuitId: 1

    // --- 左侧及中间 Aliases ---
    property alias testStartStop: testStartStop
    property alias testStrategyCombo: testStrategyCombo
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
    property alias refStrategyCombo: refStrategyCombo
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

    // --- 右侧被试品(Cable) Aliases ---
    property alias cableComboBox: cableComboBox
    property alias lblCoreDia: lblCoreDia
    property alias lblCoreMat: lblCoreMat
    property alias lblInsThick: lblInsThick
    property alias lblInsMat: lblInsMat
    property alias lblVoltage: lblVoltage
    property alias lblFormat: lblFormat
    property alias lblNotes: lblNotes

    property alias applyLeftBtn: groupLeft.applyButton
    property alias applyMidBtn: groupMid.applyButton
    property alias applyRightBtn: groupRight.applyButton
    property alias restoreLeftBtn: groupLeft.restoreButton
    property alias restoreMidBtn: groupMid.restoreButton
    property alias restoreRightBtn: groupRight.restoreButton

    property alias messagePopup: messagePopup

    RowLayout {
        anchors.fill: parent
        spacing: 20

        // =================================================
        // 左侧：试验回路 (占比: 3)
        // =================================================
        SettingsGroup {
            id: groupLeft
            title: "试验回路" + root.circuitId
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: 300

            StartStopPanel {
                id: testStartStop
                Layout.fillWidth: true
                loopId: (root.circuitId - 1) * 2 + 1
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: 10
                Label { text: "自动策略:"; color: Theme.textColor; font: Theme.defaultFont; Layout.preferredWidth: 120; horizontalAlignment: Text.AlignRight }
                ComboBox {
                    id: testStrategyCombo
                    Layout.preferredWidth: 150
                    Layout.preferredHeight: 40
                    font: Theme.defaultFont
                    model: ["恒流模式 (1)", "温控模式 (2)"]
                }
                Item { Layout.fillWidth: true }
            }

            Label { text: "电流设置"; color: Theme.orange; font: Theme.defaultFont; Layout.alignment: Qt.AlignHCenter }
            ColumnLayout {
                Layout.fillWidth: true
                spacing: 10
                SettingInput { id: testStartCurrent; labelText: "起始电流:"; unitText: "A" }
                SettingInput { id: testMaxCurrent; labelText: "最大电流:"; unitText: "A" }
                SettingInput { id: testChangePercent; labelText: "恒流误差:"; unitText: "A"; inputValidator: IntValidator { bottom: 2; top: 20 } }
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
                                id: testHeatInputHour; anchors.fill: parent
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
                                id: testHeatInputMin; anchors.fill: parent
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
        // 中间：模拟回路 (占比: 3)
        // =================================================
        SettingsGroup {
            id: groupMid
            title: "模拟回路" + root.circuitId
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: 300

            StartStopPanel {
                id: refStartStop
                Layout.fillWidth: true
                loopId: (root.circuitId - 1) * 2 + 2
            }

            RowLayout {
                Layout.fillWidth: true
                spacing: 10
                Label { text: "自动策略:"; color: Theme.textColor; font: Theme.defaultFont; Layout.preferredWidth: 120; horizontalAlignment: Text.AlignRight }
                ComboBox {
                    id: refStrategyCombo
                    Layout.preferredWidth: 150
                    Layout.preferredHeight: 40
                    font: Theme.defaultFont
                    model: ["恒流模式 (1)", "温控模式 (2)"]
                }
                Item { Layout.fillWidth: true }
            }

            Label { text: "电流设置"; color: Theme.orange; font: Theme.defaultFont; Layout.alignment: Qt.AlignHCenter }
            ColumnLayout {
                Layout.fillWidth: true
                spacing: 10
                SettingInput { id: refStartCurrent; labelText: "起始电流:"; unitText: "A" }
                SettingInput { id: refMaxCurrent; labelText: "最大电流:"; unitText: "A" }
                SettingInput { id: refChangePercent; labelText: "恒流误差:"; unitText: "A"; inputValidator: IntValidator { bottom: 2; top: 20 } }
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
                                id: refHeatInputHour; anchors.fill: parent
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
                                id: refHeatInputMin; anchors.fill: parent
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
        // 右侧 (基于 Cable 对象的被试品区域, 占比: 2)
        // =================================================
        ColumnLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.preferredWidth: 200

            SettingsGroup {
                id: groupRight
                title: "被试品"
                Layout.fillWidth: true
                Layout.fillHeight: true

                // 1. 顶部：电缆名称 (绑定高度，与左侧的 StartStopPanel 绝对平行)
                Item {
                    Layout.fillWidth: true
                    Layout.preferredHeight: testStartStop.height > 0 ? testStartStop.height : 60

                    RowLayout {
                        anchors.centerIn: parent
                        width: parent.width - 20
                        Label { text: "电缆名称:"; color: Theme.textColor; font: Theme.defaultFont; Layout.preferredWidth: 80; horizontalAlignment: Text.AlignRight }
                        ComboBox {
                            id: cableComboBox
                            Layout.fillWidth: true
                            Layout.preferredHeight: 40
                            font: Theme.defaultFont
                            textRole: "name"
                        }
                    }
                }

                // 2. 占位符：与左侧“自动策略”同高度(隐式高度40)，推动底部文本同步下降
                Item {
                    Layout.fillWidth: true
                    Layout.preferredHeight: 50
                }

                // 3. 中间：小标题 (天然与左侧的“电流设置”水平对齐)
                Label { text: "电缆属性"; color: Theme.orange; font: Theme.defaultFont; Layout.alignment: Qt.AlignHCenter }

                // 4. 底部：只读属性面板 (强制绑定一致的行高度，行间距直接与 SettingInput 平齐)
                Item {
                    Layout.fillWidth: true
                    Layout.fillHeight: true

                    ScrollView {
                        anchors.fill: parent
                        clip: true
                        ScrollBar.vertical.policy: ScrollBar.AsNeeded

                        ColumnLayout {
                            width: parent.width
                            spacing: 10 // 完全一致的 10 空隙

                            // 自动跟随左侧电流设置的高度，如果没有初始化则保底40
                            property real rowH: testStartCurrent.height > 0 ? testStartCurrent.height : 40

                            // 每行左右皆用 Item{ Layout.fillWidth } 撑开，迫使中间的两列 Label 紧凑并居中
                            RowLayout {
                                Layout.fillWidth: true; Layout.preferredHeight: parent.rowH
                                Item { Layout.fillWidth: true }
                                Label { text: "电芯直径:"; color: Theme.textColor; font: Theme.defaultFont; Layout.preferredWidth: 90; horizontalAlignment: Text.AlignRight; verticalAlignment: Text.AlignVCenter }
                                Label { id: lblCoreDia; text: "-"; color: Theme.valueColor; font: Theme.defaultFont; Layout.preferredWidth: 110; horizontalAlignment: Text.AlignLeft; verticalAlignment: Text.AlignVCenter }
                                Item { Layout.fillWidth: true }
                            }

                            RowLayout {
                                Layout.fillWidth: true; Layout.preferredHeight: parent.rowH
                                Item { Layout.fillWidth: true }
                                Label { text: "电芯材料:"; color: Theme.textColor; font: Theme.defaultFont; Layout.preferredWidth: 90; horizontalAlignment: Text.AlignRight; verticalAlignment: Text.AlignVCenter }
                                Label { id: lblCoreMat; text: "-"; color: Theme.valueColor; font: Theme.defaultFont; Layout.preferredWidth: 110; horizontalAlignment: Text.AlignLeft; verticalAlignment: Text.AlignVCenter }
                                Item { Layout.fillWidth: true }
                            }

                            RowLayout {
                                Layout.fillWidth: true; Layout.preferredHeight: parent.rowH
                                Item { Layout.fillWidth: true }
                                Label { text: "绝缘厚度:"; color: Theme.textColor; font: Theme.defaultFont; Layout.preferredWidth: 90; horizontalAlignment: Text.AlignRight; verticalAlignment: Text.AlignVCenter }
                                Label { id: lblInsThick; text: "-"; color: Theme.valueColor; font: Theme.defaultFont; Layout.preferredWidth: 110; horizontalAlignment: Text.AlignLeft; verticalAlignment: Text.AlignVCenter }
                                Item { Layout.fillWidth: true }
                            }

                            RowLayout {
                                Layout.fillWidth: true; Layout.preferredHeight: parent.rowH
                                Item { Layout.fillWidth: true }
                                Label { text: "绝缘材料:"; color: Theme.textColor; font: Theme.defaultFont; Layout.preferredWidth: 90; horizontalAlignment: Text.AlignRight; verticalAlignment: Text.AlignVCenter }
                                Label { id: lblInsMat; text: "-"; color: Theme.valueColor; font: Theme.defaultFont; Layout.preferredWidth: 110; horizontalAlignment: Text.AlignLeft; verticalAlignment: Text.AlignVCenter }
                                Item { Layout.fillWidth: true }
                            }

                            RowLayout {
                                Layout.fillWidth: true; Layout.preferredHeight: parent.rowH
                                Item { Layout.fillWidth: true }
                                Label { text: "电压等级:"; color: Theme.textColor; font: Theme.defaultFont; Layout.preferredWidth: 90; horizontalAlignment: Text.AlignRight; verticalAlignment: Text.AlignVCenter }
                                Label { id: lblVoltage; text: "-"; color: Theme.valueColor; font: Theme.defaultFont; Layout.preferredWidth: 110; horizontalAlignment: Text.AlignLeft; verticalAlignment: Text.AlignVCenter }
                                Item { Layout.fillWidth: true }
                            }

                            RowLayout {
                                Layout.fillWidth: true; Layout.preferredHeight: parent.rowH
                                Item { Layout.fillWidth: true }
                                Label { text: "系统制式:"; color: Theme.textColor; font: Theme.defaultFont; Layout.preferredWidth: 90; horizontalAlignment: Text.AlignRight; verticalAlignment: Text.AlignVCenter }
                                Label { id: lblFormat; text: "-"; color: Theme.valueColor; font: Theme.defaultFont; Layout.preferredWidth: 110; horizontalAlignment: Text.AlignLeft; verticalAlignment: Text.AlignVCenter }
                                Item { Layout.fillWidth: true }
                            }

                            // 备注采用 minimumHeight, 允许文本由于太长自动换行扩充高度
                            RowLayout {
                                Layout.fillWidth: true; Layout.minimumHeight: parent.rowH
                                Item { Layout.fillWidth: true }
                                Label { text: "备注:"; color: Theme.textColor; font: Theme.defaultFont; Layout.preferredWidth: 90; horizontalAlignment: Text.AlignRight; verticalAlignment: Text.AlignVCenter }
                                Label { id: lblNotes; text: "-"; color: Theme.valueColor; font: Theme.defaultFont; Layout.preferredWidth: 110; horizontalAlignment: Text.AlignLeft; verticalAlignment: Text.AlignVCenter; wrapMode: Text.Wrap }
                                Item { Layout.fillWidth: true }
                            }
                        }
                    }
                }
            }
        }
    }
    MessagePopup { id: messagePopup }
}
