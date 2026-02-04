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

            // 内部不需要再写 Spacer 了，Group 自己会居中

            SettingInput {
                id: sampleIntervalInput
                labelText: "采样间隔:"
                unitText: "秒"
            }
            SettingInput {
                id: recordIntervalInput
                labelText: "记录间隔:"
                unitText: "分"
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

            SettingInput { id: mainOverCurrentInput; labelText: "过流保护:"; unitText: "A" }
            SettingInput { id: mainOverVoltageInput; labelText: "过压保护:"; unitText: "V" }
            SettingInput { id: mainVolUpSpeedInput; labelText: "升压速度:"; unitText: "%" }
            SettingInput { id: mainVolDownSpeedInput; labelText: "降压速度:"; unitText: "%" }
            Item{Layout.fillHeight: true}
            SettingSwitch { id: mainProtectModeSwitch; labelText: "过压模式:"; leftText: "限幅"; rightText: "分闸" }
            Item{Layout.fillHeight: true}
        }

        // === 右侧：辅调压器 ===
        SettingsGroup {
            id: auxGroup
            Layout.fillWidth: true
            title: "辅调压器"

            SettingInput { id: auxOverCurrentInput; labelText: "过流保护:"; unitText: "A" }
            SettingInput { id: auxOverVoltageInput; labelText: "过压保护:"; unitText: "V" }
            SettingInput { id: auxVolUpSpeedInput; labelText: "升压速度:"; unitText: "%" }
            SettingInput { id: auxVolDownSpeedInput; labelText: "降压速度:"; unitText: "%" }
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
