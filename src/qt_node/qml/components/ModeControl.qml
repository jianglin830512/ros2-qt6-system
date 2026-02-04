import QtQuick
import qt_node 1.0

ModeControlForm {
    id: control

    property int circuitId: 0
    property int controlMode: 0 // 从外部传入 rosProxy.circuitStatusX.control_mode

    Component.onCompleted: {
        control.title = "回路" + circuitId;
    }

    // 根据传入的 currentMode 更新按钮的指示灯状态
    onControlModeChanged: {
        manualButton.indicatorOn = (controlMode === QtNodeConstants.CMD_CIRCUIT_SET_MANUAL_MODE);
        constCurrentButton.indicatorOn = (controlMode === QtNodeConstants.CMD_CIRCUIT_SET_CONST_CURRENT_MODE);
        tempControlButton.indicatorOn = (controlMode === QtNodeConstants.CMD_CIRCUIT_SET_TEMP_CONTROL_MODE);
    }

    // 按钮点击时发送命令
    manualButton.onSendCommand: rosProxy.sendCircuitModeCommand(circuitId, QtNodeConstants.CMD_CIRCUIT_SET_MANUAL_MODE)
    constCurrentButton.onSendCommand: rosProxy.sendCircuitModeCommand(circuitId, QtNodeConstants.CMD_CIRCUIT_SET_CONST_CURRENT_MODE)
    tempControlButton.onSendCommand: rosProxy.sendCircuitModeCommand(circuitId, QtNodeConstants.CMD_CIRCUIT_SET_TEMP_CONTROL_MODE)
}
