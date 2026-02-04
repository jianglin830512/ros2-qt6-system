// 文件: qml/components/LoopStatus.qml
// [已修复]

import QtQuick
import qt_node 1.0

LoopStatusForm {
    id: control

    // --- Public API ---
    property int circuitId: 1
    property bool isSimulated: false
    property string title: "回路"
    property var loopStatusData: null
    property var loopSettingsData: null
    // 用于接收自定义温度标题的列表
    property var temperatureTitles: []

    // ... 辅助函数和 onCompleted 保持不变 ...
    function getHeatingStatusText() {
        if (!loopSettingsData || !loopSettingsData.enabled) return "";
        return "加热中";
    }
    Component.onCompleted: {
        control.titleLabel.text = title
    }

    // 监听 loopStatusData 属性本身的变化
    onLoopStatusDataChanged: {
        // 直接使用 loopStatusData，不再有 .test_loop 或 .ref_loop
        if (!loopStatusData) return;
        control.measureCurrentLabel.text = loopStatusData.current.toFixed(0)
        control.heatRemainValue = (loopStatusData.remaining_heating_time_sec / 60).toFixed(0)
        control.cycleRemainValue = loopStatusData.remaining_cycle_count
        control.closeBreakerButton.indicatorOn = loopStatusData.breaker_closed_switch_ack
        control.openBreakerButton.indicatorOn = loopStatusData.breaker_opened_switch_ack
        control.statusLabel.text = loopStatusData.breaker_closed_switch_ack ? "加热中" : "冷却中"
        control.statusLabel.color = loopStatusData.breaker_closed_switch_ack ? "red" : "green"

        var temps = []
        // 遍历传入的标题列表
        for (var i = 0; i < temperatureTitles.length; ++i) {
            var title = temperatureTitles[i];
            // 从数据源获取对应索引的温度值，如果不存在则显示 N/A
            var value = loopStatusData.temperature_array[i] !== undefined
                    ? loopStatusData.temperature_array[i].toFixed(1)
                    : "N/A";

            // 创建模型对象并添加到数组
            temps.push({ titleName: title, value: value });
        }
        control.tempRepeater.model = temps; // 将新模型赋值给 Repeater
    }

    // 监听 loopSettingsData 属性本身的变化
    onLoopSettingsDataChanged: {
        // 直接使用 loopSettingsData
        if (!loopSettingsData) return;
        control.setCurrentLabel.text = loopSettingsData.start_current_a
        control.heatSetValue = (loopSettingsData.heating_duration_sec / 60).toFixed(0)
        control.cycleSetValue = loopSettingsData.cycle_count
        control.enableLabel.text = loopSettingsData.enabled ? "启用" : "停用"
        control.enableLabel.color = loopSettingsData.enabled ? "lime" : "#888888"
        control.statusLabel.text = getHeatingStatusText()
        control.statusLabel.visible = loopSettingsData.enabled
    }

    // --- 命令发送 (这部分无需修改) ---
    closeBreakerButton.onSendCommand: {
        var cmd = isSimulated ? QtNodeConstants.CMD_CIRCUIT_SIM_BREAKER_CLOSE : QtNodeConstants.CMD_CIRCUIT_TEST_BREAKER_CLOSE;
        rosProxy.sendCircuitBreakerCommand(circuitId, cmd);
    }
    openBreakerButton.onSendCommand: {
        var cmd = isSimulated ? QtNodeConstants.CMD_CIRCUIT_SIM_BREAKER_OPEN : QtNodeConstants.CMD_CIRCUIT_TEST_BREAKER_OPEN;
        rosProxy.sendCircuitBreakerCommand(circuitId, cmd);
    }
}
