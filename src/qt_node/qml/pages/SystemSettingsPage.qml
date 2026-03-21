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
            mainVolUpSpeedInput.settingValue = rosProxy.qmlRegulatorSettings1.voltage_up_speed_percent;
            mainVolDownSpeedInput.settingValue = rosProxy.qmlRegulatorSettings1.voltage_down_speed_percent;
        }
    }

    function syncAuxRegInputs() {
        if (rosProxy.qmlRegulatorSettings2) {
            auxOverCurrentInput.settingValue = rosProxy.qmlRegulatorSettings2.over_current_a;
            auxVolUpSpeedInput.settingValue = rosProxy.qmlRegulatorSettings2.voltage_up_speed_percent;
            auxVolDownSpeedInput.settingValue = rosProxy.qmlRegulatorSettings2.voltage_down_speed_percent;
        }
    }

    function syncAllInputs() {
        syncSystemInputs();
        syncMainRegInputs();
        syncAuxRegInputs();
    }

    function clampAndApplyInput(inputComponent, min, max) {
        var val = parseInt(inputComponent.settingValue);
        // 如果输入为空或非法字符，默认为最小值
        if (isNaN(val)) val = min;
        // 限制下限和上限
        if (val < min) val = min;
        if (val > max) val = max;

        // 将修正后的合法值直接反填回 UI 输入框，让用户直观看到被限制了
        inputComponent.settingValue = val.toString();

        return val;
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
                page.mainVolUpSpeedInput.currentValue = data.voltage_up_speed_percent.toString()
                page.mainVolDownSpeedInput.currentValue = data.voltage_down_speed_percent.toString()
            }
        }
    }

    Connections {
        target: rosProxy
        function onQmlRegulatorSettings2Changed() {
            var data = rosProxy.qmlRegulatorSettings2
            if(data) {
                page.auxOverCurrentInput.currentValue = data.over_current_a.toString()
                page.auxVolUpSpeedInput.currentValue = data.voltage_up_speed_percent.toString()
                page.auxVolDownSpeedInput.currentValue = data.voltage_down_speed_percent.toString()
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

            // 限制：记录间隔 1 ~ 10 分钟
            sysData.record_interval_min = clampAndApplyInput(recordIntervalInput, 1, 10);
            sysData.keep_record_on_shutdown = keepRecordSwitch.isSettingOn

            rosProxy.setSystemSettings(sysData)
        }
    }

    // 按钮 2: 主调压器
    applyMainBtn.onClicked: {
        if (rosProxy.qmlRegulatorSettings1) {
            var reg1Data = rosProxy.qmlRegulatorSettings1

            // 限制：过流 100~500, 升压 10~100, 降压 10~100
            // 未在此处赋值的过压相关属性，会在对象中保持原有值被下发写入
            reg1Data.over_current_a = clampAndApplyInput(mainOverCurrentInput, 100, 500);
            reg1Data.voltage_up_speed_percent = clampAndApplyInput(mainVolUpSpeedInput, 10, 100);
            reg1Data.voltage_down_speed_percent = clampAndApplyInput(mainVolDownSpeedInput, 10, 100);

            rosProxy.setRegulatorSettings(1, reg1Data)
        }
    }

    // 按钮 3: 辅调压器
    applyAuxBtn.onClicked: {
        if (rosProxy.qmlRegulatorSettings2) {
            var reg2Data = rosProxy.qmlRegulatorSettings2

            // 限制：过流 100~500, 升压 10~100, 降压 10~100
            // 未在此处赋值的过压相关属性，会在对象中保持原有值被下发写入
            reg2Data.over_current_a = clampAndApplyInput(auxOverCurrentInput, 100, 500);
            reg2Data.voltage_up_speed_percent = clampAndApplyInput(auxVolUpSpeedInput, 10, 100);
            reg2Data.voltage_down_speed_percent = clampAndApplyInput(auxVolDownSpeedInput, 10, 100);

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
