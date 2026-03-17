import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0
import "../components"

Item {
    id: root

    // --- 输入框 Aliases (保持不变) ---
    property alias sampleIntervalInput: sampleIntervalInput
    property alias recordIntervalInput: recordIntervalInput
    property alias keepRecordSwitch: keepRecordSwitch

    property alias mainOverCurrentInput: mainOverCurrentInput
    property alias mainOverVoltageInput: mainOverVoltageInput
    property alias mainVolUpSpeedInput: mainVolUpSpeedInput
    property alias mainVolDownSpeedInput: mainVolDownSpeedInput
    property alias mainProtectModeSwitch: mainProtectModeSwitch

    property alias auxOverCurrentInput: auxOverCurrentInput
    property alias auxOverVoltageInput: auxOverVoltageInput
    property alias auxVolUpSpeedInput: auxVolUpSpeedInput
    property alias auxVolDownSpeedInput: auxVolDownSpeedInput
    property alias auxProtectModeSwitch: auxProtectModeSwitch

    // --- 按钮 Aliases  ---
    property alias applySystemBtn: systemGroup.applyButton
    property alias applyMainBtn: mainGroup.applyButton
    property alias applyAuxBtn: auxGroup.applyButton

    property alias restoreSystemBtn: systemGroup.restoreButton
    property alias restoreMainBtn: mainGroup.restoreButton
    property alias restoreAuxBtn: auxGroup.restoreButton

    // --- 弹窗 Alias ---
    property alias messagePopup: messagePopup

    RowLayout {
        anchors.fill: parent
        spacing: 30

        // === 左侧：系统参数 ===
        SettingsGroup {
            id: systemGroup // 给 Group 加 ID，方便别名引用
            Layout.fillWidth: true
            title: "系统参数"

            SettingInput {
                id: sampleIntervalInput
                labelText: "采样间隔:"
                unitText: "秒"
                visible: false // 1. 隐藏采样间隔设置
            }
            SettingInput {
                id: recordIntervalInput
                labelText: "记录间隔:"
                unitText: "分"
                // 2. 记录间隔设定最小值1，最大值10
                inputValidator: IntValidator { bottom: 1; top: 10 }
            }
            Item{Layout.preferredHeight: 40}  // 占位
            Item{Layout.preferredHeight: 40}  // 占位
            Item{Layout.fillHeight: true}
            SettingSwitch {
                id: keepRecordSwitch
                labelText: "停机保持记录"
                leftText: "禁用"
                rightText: "启用"
            }
            Item{Layout.fillHeight: true}
        }

        // === 中间：主调压器 ===
        SettingsGroup {
            id: mainGroup
            Layout.fillWidth: true
            title: "主调压器"

            SettingInput {
                id: mainOverCurrentInput; labelText: "过流保护:"; unitText: "A"
                inputValidator: IntValidator { bottom: 100; top: 500 } // 3. 最小值100，最大值500
            }
            SettingInput {
                id: mainOverVoltageInput; labelText: "过压保护:"; unitText: "V"
                inputValidator: IntValidator { bottom: 100; top: 500 } // 3. 最小值100，最大值500
            }
            SettingInput {
                id: mainVolUpSpeedInput; labelText: "升压速度:"; unitText: "%"
                inputValidator: IntValidator { bottom: 10; top: 100 }  // 4. 最小值10，最大值100
            }
            SettingInput {
                id: mainVolDownSpeedInput; labelText: "降压速度:"; unitText: "%"
                inputValidator: IntValidator { bottom: 10; top: 100 }  // 4. 最小值10，最大值100
            }
            Item{Layout.fillHeight: true}
            SettingSwitch { id: mainProtectModeSwitch; labelText: "过压模式:"; leftText: "限幅"; rightText: "分闸" }
            Item{Layout.fillHeight: true}
        }

        // === 右侧：辅调压器 ===
        SettingsGroup {
            id: auxGroup
            Layout.fillWidth: true
            title: "辅调压器"

            SettingInput {
                id: auxOverCurrentInput; labelText: "过流保护:"; unitText: "A"
                inputValidator: IntValidator { bottom: 100; top: 500 } // 3. 最小值100，最大值500
            }
            SettingInput {
                id: auxOverVoltageInput; labelText: "过压保护:"; unitText: "V"
                inputValidator: IntValidator { bottom: 100; top: 500 } // 3. 最小值100，最大值500
            }
            SettingInput {
                id: auxVolUpSpeedInput; labelText: "升压速度:"; unitText: "%"
                inputValidator: IntValidator { bottom: 10; top: 100 }  // 4. 最小值10，最大值100
            }
            SettingInput {
                id: auxVolDownSpeedInput; labelText: "降压速度:"; unitText: "%"
                inputValidator: IntValidator { bottom: 10; top: 100 }  // 4. 最小值10，最大值100
            }
            Item{Layout.fillHeight: true}
            SettingSwitch { id: auxProtectModeSwitch; labelText: "过压模式:"; leftText: "限幅"; rightText: "分闸" }
            Item{Layout.fillHeight: true}
        }
    }

    // 通用弹窗
    MessagePopup {
        id: messagePopup
    }
}
