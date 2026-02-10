import QtQuick
import qt_node 1.0

LoopStatusForm {
    id: control

    property int circuitId: 1
    property bool isSimulated: false
    property string title: "回路"
    property var loopStatusData: null
    property var loopSettingsData: null
    property var temperatureTitles: []
    property int controlMode: 0  // 接收控制模式

    Component.onCompleted: {
        control.titleLabel.text = title
    }

    // --- [绑定] 设定电流 ---
    setCurrentLabel.text: {
        if (loopSettingsData && loopSettingsData.start_current_a !== undefined) {
            return String(loopSettingsData.start_current_a);
        }
        return "0";
    }

    // ==========================================================
    // 【新增】 绑定 加热时长 和 循环次数 到设定值显示
    // ==========================================================

    // 绑定加热设定值 (ROS类型: builtin_interfaces/Duration -> C++: int heating_duration_sec)
    // UI单位为分钟，所以需要除以 60
    heatSetValue: {
        if (loopSettingsData && loopSettingsData.heating_duration_sec !== undefined) {
            return (loopSettingsData.heating_duration_sec / 60).toFixed(0);
        }
        return "0";
    }

    // 绑定循环次数设定值 (ROS类型: int32 cycle_count -> C++: int cycle_count)
    cycleSetValue: {
        if (loopSettingsData && loopSettingsData.cycle_count !== undefined) {
            return String(loopSettingsData.cycle_count);
        }
        return "0";
    }
    // ==========================================================


    // --- 监听状态数据 (保持不变) ---
    onLoopStatusDataChanged: {
        if (!loopStatusData) {
            control.measureCurrentLabel.text = "0";
            return;
        }
        // 测量电流
        control.measureCurrentLabel.text = loopStatusData.current ? loopStatusData.current.toFixed(0) : "0"

        // 剩余时间 (秒 -> 分)
        control.heatRemainValue = loopStatusData.remaining_heating_time_sec ? (loopStatusData.remaining_heating_time_sec / 60).toFixed(0) : "0"
        // 剩余循环次数
        control.cycleRemainValue = loopStatusData.remaining_cycle_count ? loopStatusData.remaining_cycle_count : "0"

        control.closeBreakerButton.indicatorOn = loopStatusData.breaker_closed_switch_ack
        control.openBreakerButton.indicatorOn = loopStatusData.breaker_opened_switch_ack

        var temps = []
        for (var i = 0; i < temperatureTitles.length; ++i) {
            var title = temperatureTitles[i];
            var value = (loopStatusData.temperature_array && loopStatusData.temperature_array[i] !== undefined)
                    ? loopStatusData.temperature_array[i].toFixed(1)
                    : "N/A";
            temps.push({ titleName: title, value: value });
        }
        control.tempRepeater.model = temps;
    }

    // --- 监听设置数据 (启用状态/颜色/文本逻辑保持不变) ---
    enableLabel.text: {
        if (loopSettingsData && loopSettingsData.enabled) {
            return "启用";
        }
        return "停用";
    }

    enableLabel.color: {
        if (loopSettingsData && loopSettingsData.enabled) {
            return "lime";
        }
        return "#888888";
    }

    statusLabel.text: {
        if (loopStatusData && loopStatusData.is_heat) {
            return "加热中";
        }
        return "冷却中";
    }

    statusLabel.color: {
        if (loopStatusData && loopStatusData.is_heat) {
            return "red";
        }
        return "green";
    }

    statusLabel.visible: {
        return (loopSettingsData && loopSettingsData.enabled);
    }

    closeBreakerButton.onSendCommand: {
        var cmd = isSimulated ? QtNodeConstants.CMD_CIRCUIT_SIM_BREAKER_CLOSE : QtNodeConstants.CMD_CIRCUIT_TEST_BREAKER_CLOSE;
        rosProxy.sendCircuitBreakerCommand(circuitId, cmd);
    }
    openBreakerButton.onSendCommand: {
        var cmd = isSimulated ? QtNodeConstants.CMD_CIRCUIT_SIM_BREAKER_OPEN : QtNodeConstants.CMD_CIRCUIT_TEST_BREAKER_OPEN;
        rosProxy.sendCircuitBreakerCommand(circuitId, cmd);
    }

    // 绑定按钮遮罩状态
    // 只有当：1. 回路已启用 且 2. 不是手动模式时，才遮挡按钮。
    // (如果回路未启用，全局遮罩已经生效了，所以这里只需关心模式)
    isButtonsBlocked: (controlMode !== QtNodeConstants.CMD_CIRCUIT_SET_MANUAL_MODE)
}
