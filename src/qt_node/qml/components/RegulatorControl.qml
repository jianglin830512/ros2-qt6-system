import QtQuick
import qt_node 1.0
import "./"

RegulatorControlForm {
    id: control

    // --- Public API ---
    property int regulatorId: 1
    property string title: "调压器"
    property var statusData: null
    property int controlMode: 0

    // --- Initialization ---
    Component.onCompleted: {
        control.titleLabel.text = title
        control.closeBreakerButton.labelText = (regulatorId === 1 ? "主" : "辅") + "合闸"
        control.openBreakerButton.labelText = (regulatorId === 1 ? "主" : "辅") + "分闸"
        control.voltageUpButton.labelText = (regulatorId === 1 ? "主" : "辅") + "升压"
        control.voltageDownButton.labelText = (regulatorId === 1 ? "主" : "辅") + "降压"

        // 初始化 Y 轴量程
        // 主调压器(1)为 450A，辅调压器(2)为 225A
        control.currentAxis.max = (regulatorId === 1) ? 450 : 225;
        control.voltageAxis.max = 450;
    }

    // --- Data Binding ---
    Connections {
        target: control
        function onStatusDataChanged() {
            if (!statusData) return;

            var vol = statusData.voltage_reading;
            var cur = statusData.current_reading;

            // 1. 更新文本和状态
            control.voltageLabel.text = vol.toFixed(1) + " V"
            control.currentLabel.text = cur.toFixed(1) + " A"
            control.overCurrentLabel.visible = statusData.over_current_on
            control.upArrow.visible = statusData.voltage_direction === 1
            control.downArrow.visible = statusData.voltage_direction === -1
            control.closeBreakerButton.indicatorOn = statusData.breaker_closed_switch_ack
            control.openBreakerButton.indicatorOn = statusData.breaker_opened_switch_ack
            control.voltageUpButton.indicatorOn = statusData.upper_limit_switch_on
            control.voltageDownButton.indicatorOn = statusData.lower_limit_switch_on

            // 2. 动态更新图表
            updateChart(vol, cur);
        }
    }

    function updateChart(vol, cur) {
        // --- 电压计算 (Max 450V) ---
        var maxV = 450.0;
        var warnV = maxV * 0.8;   // 360V
        var dangerV = maxV * 0.9; // 405V

        var baseV_val = 0, warnV_val = 0, dangerV_val = 0;

        if (vol <= warnV) {
            baseV_val = vol;
        } else if (vol <= dangerV) {
            baseV_val = warnV;
            warnV_val = vol - warnV;
        } else {
            baseV_val = warnV;
            warnV_val = dangerV - warnV;
            dangerV_val = vol - dangerV;
        }

        // 索引 0 表示第一列（Category "V"）
        control.volBaseSet.replace(0, baseV_val);
        control.volWarnSet.replace(0, warnV_val);
        control.volDangerSet.replace(0, dangerV_val);


        // --- 电流计算 ---
        var maxA = (regulatorId === 1) ? 450.0 : 225.0;
        var warnA = maxA * 0.8;
        var dangerA = maxA * 0.9;

        var baseA_val = 0, warnA_val = 0, dangerA_val = 0;

        if (cur <= warnA) {
            baseA_val = cur;
        } else if (cur <= dangerA) {
            baseA_val = warnA;
            warnA_val = cur - warnA;
        } else {
            baseA_val = warnA;
            warnA_val = dangerA - warnA;
            dangerA_val = cur - dangerA;
        }

        // 索引 1 表示第二列（Category "A"）
        control.curBaseSet.replace(1, baseA_val);
        control.curWarnSet.replace(1, warnA_val);
        control.curDangerSet.replace(1, dangerA_val);
    }

    // --- Command Sending ---
    closeBreakerButton.onSendCommand: {
        rosProxy.sendRegulatorBreakerCommand(regulatorId, QtNodeConstants.CMD_REGULATOR_BREAKER_CLOSE)
    }

    openBreakerButton.onSendCommand: {
        rosProxy.sendRegulatorBreakerCommand(regulatorId, QtNodeConstants.CMD_REGULATOR_BREAKER_OPEN)
    }

    voltageUpButton.onSendCommand: {
        rosProxy.sendRegulatorOperationCommand(regulatorId, QtNodeConstants.CMD_REGULATOR_VOLTAGE_UP)
    }
    voltageUpButton.onReleasedCommand: {
        rosProxy.sendRegulatorOperationCommand(regulatorId, QtNodeConstants.CMD_REGULATOR_VOLTAGE_STOP)
    }

    voltageDownButton.onSendCommand: {
        rosProxy.sendRegulatorOperationCommand(regulatorId, QtNodeConstants.CMD_REGULATOR_VOLTAGE_DOWN)
    }
    voltageDownButton.onReleasedCommand: {
        rosProxy.sendRegulatorOperationCommand(regulatorId, QtNodeConstants.CMD_REGULATOR_VOLTAGE_STOP)
    }

    isBlocked: (rosProxy.qmlSystemSettings && rosProxy.qmlSystemSettings.auto_on)
}
