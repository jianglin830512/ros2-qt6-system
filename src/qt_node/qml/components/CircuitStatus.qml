import QtQuick
import "./"

CircuitStatusForm {
    id: control

    // --- Public API ---
    property int circuitId: 1
    property var statusData: null
    property var settingsData: null

    // --- [核心修改] 直接在这里进行声明式绑定 ---

    // -- 为 testLoop 子组件绑定属性 --
    testLoop.circuitId: control.circuitId
    testLoop.isSimulated: false
    testLoop.title: "试验回路" + control.circuitId
    testLoop.loopStatusData: statusData ? statusData.test_loop : null
    testLoop.loopSettingsData: settingsData ? settingsData.test_loop : null
    testLoop.temperatureTitles : [
        "通道01", "通道02", "通道03", "通道04", "通道05", "通道06", "通道07", "通道08",
        "通道09", "通道10", "通道11", "通道12", "通道13", "通道14", "通道15", "通道16"
    ]

    // -- 为 refLoop 子组件绑定属性 --
    refLoop.circuitId: control.circuitId
    refLoop.isSimulated: true
    refLoop.title: "模拟回路" + control.circuitId
    refLoop.loopStatusData: statusData ? statusData.ref_loop : null
    refLoop.loopSettingsData: settingsData ? settingsData.ref_loop : null
    refLoop.temperatureTitles : [
        "导体01", "导体02", "导体03", "护套01", "护套02", "护套03", "通道07", "通道08"
    ]

    // -- 为 modeControl 子组件绑定属性 --
    modeControl.circuitId: control.circuitId
    modeControl.controlMode: statusData ? statusData.control_mode : 0
    modeControl.title: "回路" + control.circuitId

}
