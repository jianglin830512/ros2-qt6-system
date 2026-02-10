import QtQuick
import qt_node 1.0
import "./"

RegulatorControlForm {
    id: control

    // --- Public API ---
    property int regulatorId: 1
    property string title: "调压器"
    property var statusData: null // Expects rosProxy.regulatorStatus1, etc.
    property int controlMode: 0  // 接收控制模式

    // --- Initialization ---
    Component.onCompleted: {
        control.titleLabel.text = title
        control.closeBreakerButton.labelText = (regulatorId === 1 ? "主" : "辅") + "合闸"
        control.openBreakerButton.labelText = (regulatorId === 1 ? "主" : "辅") + "分闸"
        control.voltageUpButton.labelText = (regulatorId === 1 ? "主" : "辅") + "升压"
        control.voltageDownButton.labelText = (regulatorId === 1 ? "主" : "辅") + "降压"
    }

    // --- Data Binding ---
    Connections {
        target: control
        function onStatusDataChanged() {
            if (!statusData) return;
            control.voltageLabel.text = statusData.voltage_reading.toFixed(1) + " V"
            control.currentLabel.text = statusData.current_reading.toFixed(1) + " A"
            control.overCurrentLabel.visible = statusData.over_current_on
            control.upArrow.visible = statusData.voltage_direction === 1
            control.downArrow.visible = statusData.voltage_direction === 2 // Down is 2 in enum
            control.closeBreakerButton.indicatorOn = statusData.breaker_closed_switch_ack
            control.openBreakerButton.indicatorOn = statusData.breaker_opened_switch_ack
            control.voltageUpButton.indicatorOn = statusData.upper_limit_switch_on
            control.voltageDownButton.indicatorOn = statusData.lower_limit_switch_on
        }
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

    // 逻辑绑定：如果不是手动模式，就显示遮罩
    isBlocked: {
        // 安全检查：如果 controlMode 为 0 (未初始化)，或者等于 MANUAL (1)，则不遮挡(false)
        // 否则遮挡 (true)
        if (controlMode === QtNodeConstants.CMD_CIRCUIT_SET_MANUAL_MODE) {
            return false;
        }
        return true;
    }
}
