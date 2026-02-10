import QtQuick
import "./"

CircuitStatusForm {
    id: control

    // --- Public API ---
    property int circuitId: 1
    property var statusData: null
    property var settingsData: null

    // ==========================================================
    // 【新增】启用状态计算逻辑
    // ==========================================================

    // 安全获取试验回路启用状态 (默认 false)
    readonly property bool isTestEnabled: (settingsData && settingsData.test_loop)
                                          ? settingsData.test_loop.enabled
                                          : false

    // 安全获取模拟回路启用状态 (默认 false)
    readonly property bool isRefEnabled: (settingsData && settingsData.ref_loop)
                                         ? settingsData.ref_loop.enabled
                                         : false

    // 只要有任意一个回路启用，下方的主控制面板就应该启用
    readonly property bool isCircuitEnabled: (isTestEnabled || isRefEnabled)


    // ==========================================================
    // 【修改】声明式绑定
    // ==========================================================

    // -- 为 testLoop 子组件绑定属性 --
    testLoop.circuitId: control.circuitId
    testLoop.isSimulated: false
    testLoop.title: "试验回路" + control.circuitId
    testLoop.loopStatusData: statusData ? statusData.test_loop : null
    testLoop.loopSettingsData: settingsData ? settingsData.test_loop : null
    // 【新增】绑定遮罩状态 (取反：启用了就不遮挡，没启用就遮挡)
    testLoop.isBlocked: !control.isTestEnabled
    testLoop.controlMode: statusData ? statusData.control_mode : 0

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
    // 【新增】绑定遮罩状态
    refLoop.isBlocked: !control.isRefEnabled
    refLoop.controlMode: statusData ? statusData.control_mode : 0

    refLoop.temperatureTitles : [
        "导体01", "导体02", "导体03", "护套01", "护套02", "护套03", "通道07", "通道08"
    ]

    // -- 为 modeControl 子组件绑定属性 --
    modeControl.circuitId: control.circuitId
    modeControl.controlMode: statusData ? statusData.control_mode : 0
    modeControl.title: "回路" + control.circuitId

    // 【新增】绑定主控面板遮罩状态
    // 如果 isCircuitEnabled 为 false (都未启用)，则 isModeBlocked 为 true (遮住)
    isModeBlocked: !control.isCircuitEnabled

}
