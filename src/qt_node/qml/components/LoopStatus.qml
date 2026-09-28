import QtQuick
import qt_node 1.0

LoopStatusForm {
    id: control

    property int circuitId: 1
    property bool isSimulated: false
    property string title: "回路"
    property var loopStatusData: null
    property var loopSettingsData: null
    property int controlMode: 0
    property bool regulatorClosed: false

    property bool isHeatingNow: (loopStatusData && loopStatusData.breaker_closed_switch_ack && control.regulatorClosed)

    Component.onCompleted: {
        control.titleLabel.text = title
    }

    setCurrentLabel.text: {
        if (loopSettingsData && loopSettingsData.start_current_a !== undefined) {
            return String(loopSettingsData.start_current_a);
        }
        return "0";
    }

    heatSetValue: {
        if (loopSettingsData && loopSettingsData.heating_duration_sec !== undefined) {
            return (loopSettingsData.heating_duration_sec / 60).toFixed(0);
        }
        return "0";
    }

    cycleSetValue: {
        if (loopSettingsData && loopSettingsData.cycle_count !== undefined) {
            return String(loopSettingsData.cycle_count);
        }
        return "0";
    }

    onLoopStatusDataChanged: {
        if (!loopStatusData) {
            control.measureCurrentLabel.text = "0";
            return;
        }
        control.measureCurrentLabel.text = loopStatusData.current ? loopStatusData.current.toFixed(0) : "0"

        control.heatRemainValue = loopStatusData.remaining_heating_time_sec ? (loopStatusData.remaining_heating_time_sec / 60).toFixed(0) : "0"
        control.cycleRemainValue = loopStatusData.remaining_cycle_count ? loopStatusData.remaining_cycle_count : "0"

        control.closeBreakerButton.indicatorOn = loopStatusData.breaker_closed_switch_ack
        control.openBreakerButton.indicatorOn = loopStatusData.breaker_opened_switch_ack
    }

    enableLabel.text: {
        if (loopSettingsData && loopSettingsData.enabled) return "启用";
        return "停用";
    }

    enableLabel.color: {
        if (loopSettingsData && loopSettingsData.enabled) return Theme.statusOkColor;
        return Theme.statusDisabledColor;
    }

    statusLabel.text: isHeatingNow ? "加热中" : "冷却中"
    statusLabel.color: isHeatingNow ? Theme.statusHeatColor : Theme.statusCoolColor
    statusLabel.visible: (loopSettingsData && loopSettingsData.enabled)

    closeBreakerButton.onSendCommand: {
        var cmd = isSimulated ? QtNodeConstants.CMD_CIRCUIT_SIM_BREAKER_CLOSE : QtNodeConstants.CMD_CIRCUIT_TEST_BREAKER_CLOSE;
        rosProxy.sendCircuitBreakerCommand(circuitId, cmd);
    }
    openBreakerButton.onSendCommand: {
        var cmd = isSimulated ? QtNodeConstants.CMD_CIRCUIT_SIM_BREAKER_OPEN : QtNodeConstants.CMD_CIRCUIT_TEST_BREAKER_OPEN;
        rosProxy.sendCircuitBreakerCommand(circuitId, cmd);
    }

    isButtonsBlocked: (rosProxy.qmlSystemSettings && rosProxy.qmlSystemSettings.auto_on)
}
