import QtQuick
import QtQuick.Layouts
import "./"
import qt.theme 1.0

CircuitStatusForm {
    id: control

    // --- Public API ---
    property int circuitId: 1
    property var statusData: null
    property var settingsData: null

    property bool mainRegulatorClosed: false
    property bool auxRegulatorClosed: false

    readonly property bool isTestEnabled: (settingsData && settingsData.test_loop && settingsData.test_loop.enabled !== undefined)
                                          ? settingsData.test_loop.enabled : false
    readonly property bool isRefEnabled: (settingsData && settingsData.ref_loop && settingsData.ref_loop.enabled !== undefined)
                                         ? settingsData.ref_loop.enabled : false
    readonly property bool isCircuitEnabled: (isTestEnabled || isRefEnabled)

    // -- Test Loop 绑定 --
    testLoop.circuitId: control.circuitId
    testLoop.isSimulated: false
    testLoop.title: "试验回路" + control.circuitId
    testLoop.loopStatusData: statusData ? statusData.test_loop : null
    testLoop.loopSettingsData: settingsData ? settingsData.test_loop : null
    testLoop.isBlocked: !control.isTestEnabled
    testLoop.regulatorClosed: control.mainRegulatorClosed
    testLoop.controlMode: (statusData && statusData.control_mode !== undefined) ? statusData.control_mode : 0

    // -- Ref Loop 绑定 --
    refLoop.circuitId: control.circuitId
    refLoop.isSimulated: true
    refLoop.title: "模拟回路" + control.circuitId
    refLoop.loopStatusData: statusData ? statusData.ref_loop : null
    refLoop.loopSettingsData: settingsData ? settingsData.ref_loop : null
    refLoop.isBlocked: !control.isRefEnabled
    refLoop.regulatorClosed: control.auxRegulatorClosed
    refLoop.controlMode: (statusData && statusData.control_mode !== undefined) ? statusData.control_mode : 0

    // ==========================================
    // 回路温度展示绑定
    // ==========================================
    function updateTemperatureTable() {
        // 安全保护，防止 QML 加载早期报错
        var monitorType = (typeof rosProxy !== "undefined" && rosProxy && rosProxy.systemStatus) ? rosProxy.systemStatus.temp_monitor_type : 0;

        var numRows = (monitorType === 1) ? 4 : 5;
        var startCardIdx = (circuitId === 1) ? 1 : ((monitorType === 1) ? 5 : 6);

        var flatModel = [];
        var tempArray = (statusData && statusData.temperature_array) ? statusData.temperature_array : [];

        // 【修改点】：用 type 字段区分不同单元格的样式
        // 1. 第一行：顶部表头（无背景、无边框、仅文字）
        flatModel.push({ type: "topHeader", text: "" });
        for (var col = 1; col <= 8; col++) {
            flatModel.push({ type: "topHeader", text: col.toString() });
        }

        // 2. 构造动态数据行数
        for (var row = 0; row < numRows; row++) {
            // 第 1 列：左侧表头（带蓝色背景）
            flatModel.push({ type: "leftHeader", text: "板卡" + (startCardIdx + row) });
            // 后 8 列：数据
            for (var c = 0; c < 8; c++) {
                var idx = row * 8 + c;
                var valStr = "-";
                if (idx < tempArray.length && tempArray[idx] !== undefined) {
                    valStr = tempArray[idx].toFixed(1) + " ℃";
                }
                flatModel.push({ type: "data", text: valStr });
            }
        }
        tempRepeater.model = flatModel;
    }

    onStatusDataChanged: {
        updateTemperatureTable();
    }

    // 监听硬件型号变化
    Connections {
        target: typeof rosProxy !== "undefined" ? rosProxy : null
        function onSystemStatusChanged() {
            updateTemperatureTable();
        }
    }

    // Repeater Delegate
    tempRepeater.delegate: Rectangle {
        Layout.fillWidth: true
        Layout.fillHeight: true

        // 仅左侧行首具有蓝色背景
        color: modelData.type === "leftHeader" ? Theme.highlightColor : "transparent"

        // 如果是顶部表头（数字1-8），隐藏边框
        border.color: Theme.gridLineColor
        border.width: modelData.type === "topHeader" ? 0 : 1
        radius: 4

        Text {
            anchors.centerIn: parent
            text: modelData.text

            // 字体颜色分配：
            // 左侧表头 -> 纯白 (white)
            // 顶部数字(1-8) -> 蓝黑色 (Theme.titleColor)
            // 数据单元格 -> 默认数据色 (Theme.valueColor)
            color: modelData.type === "leftHeader" ? "white" : (modelData.type === "topHeader" ? Theme.titleColor : Theme.valueColor)

            // 顶部和左侧表头的字号大一点，数据单元格的字号小一点
            font: modelData.type === "data" ? Theme.smallLabelFont : Theme.subTitleFont
        }
    }
}
