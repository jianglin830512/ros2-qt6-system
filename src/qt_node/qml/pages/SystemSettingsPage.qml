import QtQuick
import QtQuick.Controls
import qt_node 1.0

SystemSettingsPageForm {
    id: page

    // ============================================================
    // 1. 数据同步逻辑 (Read from ROS)
    // ============================================================

    function syncSystemInputs() {
        if (rosProxy.qmlSystemSettings) {
            sampleIntervalInput.settingValue = rosProxy.qmlSystemSettings.sample_interval_sec;
            recordIntervalInput.settingValue = rosProxy.qmlSystemSettings.record_interval_min;
            keepRecordSwitch.isSettingOn = rosProxy.qmlSystemSettings.keep_record_on_shutdown;
        }
    }

    function syncMainRegInputs() {
        if (rosProxy.qmlRegulatorSettings1) {
            mainOverCurrentInput.settingValue = rosProxy.qmlRegulatorSettings1.over_current_a;
            mainOverVoltageInput.settingValue = rosProxy.qmlRegulatorSettings1.over_voltage_v;
            mainVolUpSpeedInput.settingValue = rosProxy.qmlRegulatorSettings1.voltage_up_speed_percent;
            mainVolDownSpeedInput.settingValue = rosProxy.qmlRegulatorSettings1.voltage_down_speed_percent;
            mainProtectModeSwitch.isSettingOn = rosProxy.qmlRegulatorSettings1.over_voltage_protection_mode;
        }
    }

    function syncAuxRegInputs() {
        if (rosProxy.qmlRegulatorSettings2) {
            auxOverCurrentInput.settingValue = rosProxy.qmlRegulatorSettings2.over_current_a;
            auxOverVoltageInput.settingValue = rosProxy.qmlRegulatorSettings2.over_voltage_v;
            auxVolUpSpeedInput.settingValue = rosProxy.qmlRegulatorSettings2.voltage_up_speed_percent;
            auxVolDownSpeedInput.settingValue = rosProxy.qmlRegulatorSettings2.voltage_down_speed_percent;
            auxProtectModeSwitch.isSettingOn = rosProxy.qmlRegulatorSettings2.over_voltage_protection_mode;
        }
    }

    function syncAllInputs() {
        syncSystemInputs();
        syncMainRegInputs();
        syncAuxRegInputs();
    }

    // 【新增】页面进入时自动反填
    onVisibleChanged: {
        if (visible) syncAllInputs();
    }

    // 页面加载完成后，进行一次初始化同步
    Component.onCompleted: {
        // 触发一次更新以确保数据最新
        rosProxy.qmlSystemSettingsChanged()
        rosProxy.qmlRegulatorSettings1Changed()
        rosProxy.qmlRegulatorSettings2Changed()
        syncAllInputs();
    }

    // --- 实时更新反馈值 (currentValue) ---
    // 注意：这里只更新 currentValue (实心框)，不触碰 settingValue (输入框)

    Connections {
        target: rosProxy
        function onQmlSystemSettingsChanged() {
            var data = rosProxy.qmlSystemSettings
            if(data) {
                page.sampleIntervalInput.currentValue = data.sample_interval_sec.toString()
                page.recordIntervalInput.currentValue = data.record_interval_min.toString()
                page.keepRecordSwitch.isCurrentOn = data.keep_record_on_shutdown
            }
        }
    }

    Connections {
        target: rosProxy
        function onQmlRegulatorSettings1Changed() {
            var data = rosProxy.qmlRegulatorSettings1
            if(data) {
                page.mainOverCurrentInput.currentValue = data.over_current_a.toString()
                page.mainOverVoltageInput.currentValue = data.over_voltage_v.toString()
                page.mainVolUpSpeedInput.currentValue = data.voltage_up_speed_percent.toString()
                page.mainVolDownSpeedInput.currentValue = data.voltage_down_speed_percent.toString()
                page.mainProtectModeSwitch.isCurrentOn = data.over_voltage_protection_mode
            }
        }
    }

    Connections {
        target: rosProxy
        function onQmlRegulatorSettings2Changed() {
            var data = rosProxy.qmlRegulatorSettings2
            if(data) {
                page.auxOverCurrentInput.currentValue = data.over_current_a.toString()
                page.auxOverVoltageInput.currentValue = data.over_voltage_v.toString()
                page.auxVolUpSpeedInput.currentValue = data.voltage_up_speed_percent.toString()
                page.auxVolDownSpeedInput.currentValue = data.voltage_down_speed_percent.toString()
                page.auxProtectModeSwitch.isCurrentOn = data.over_voltage_protection_mode
            }
        }
    }

    // ============================================================
    // 2. 应用设置逻辑 (Write to ROS)
    // ============================================================

    // 按钮 1: 系统参数
    applySystemBtn.onClicked: {
        if (rosProxy.qmlSystemSettings) {
            var sysData = rosProxy.qmlSystemSettings
            sysData.sample_interval_sec = parseInt(sampleIntervalInput.settingValue)
            sysData.record_interval_min = parseInt(recordIntervalInput.settingValue)
            sysData.keep_record_on_shutdown = keepRecordSwitch.isSettingOn

            rosProxy.setSystemSettings(sysData)
        }
    }

    // 按钮 2: 主调压器
    applyMainBtn.onClicked: {
        if (rosProxy.qmlRegulatorSettings1) {
            var reg1Data = rosProxy.qmlRegulatorSettings1
            reg1Data.over_current_a = parseInt(mainOverCurrentInput.settingValue)
            reg1Data.over_voltage_v = parseInt(mainOverVoltageInput.settingValue)
            reg1Data.voltage_up_speed_percent = parseInt(mainVolUpSpeedInput.settingValue)
            reg1Data.voltage_down_speed_percent = parseInt(mainVolDownSpeedInput.settingValue)
            reg1Data.over_voltage_protection_mode = mainProtectModeSwitch.isSettingOn

            rosProxy.setRegulatorSettings(1, reg1Data)
        }
    }

    // 按钮 3: 辅调压器
    applyAuxBtn.onClicked: {
        if (rosProxy.qmlRegulatorSettings2) {
            var reg2Data = rosProxy.qmlRegulatorSettings2
            reg2Data.over_current_a = parseInt(auxOverCurrentInput.settingValue)
            reg2Data.over_voltage_v = parseInt(auxOverVoltageInput.settingValue)
            reg2Data.voltage_up_speed_percent = parseInt(auxVolUpSpeedInput.settingValue)
            reg2Data.voltage_down_speed_percent = parseInt(auxVolDownSpeedInput.settingValue)
            reg2Data.over_voltage_protection_mode = auxProtectModeSwitch.isSettingOn

            rosProxy.setRegulatorSettings(2, reg2Data)
        }
    }

    // --- 还原按钮 ---
    restoreSystemBtn.onClicked: syncSystemInputs()
    restoreMainBtn.onClicked: syncMainRegInputs()
    restoreAuxBtn.onClicked: syncAuxRegInputs()

    // ============================================================
    // 3. 错误处理逻辑
    // ============================================================

    Connections {
        target: rosProxy

        function onSettingsUpdateResult(serviceName, success, message) {
            if (!success) {
                // 设置弹窗属性
                page.messagePopup.isError = true
                page.messagePopup.title = "设置失败"
                // 优化显示文本
                page.messagePopup.message = "调用服务 [" + serviceName + "] 时发生错误。\n\n原因: " + message
                page.messagePopup.open()
            } else {
                // 成功时，也可以选择弹一个绿色的提示，或者什么都不做（工业通常什么都不做）
                // 如果想弹提示，取消下面注释：
                /*
                    page.messagePopup.isError = false
                    page.messagePopup.title = "成功"
                    page.messagePopup.message = "参数设置已成功应用。"
                    page.messagePopup.open()
                    */
                console.log("Success setting: " + serviceName)
            }
        }
    }
}
