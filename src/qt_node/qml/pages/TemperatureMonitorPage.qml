import QtQuick
import qt_node 1.0
import "../components"

TemperatureMonitorPageForm {
    id: control

    // --- 主调压器 ---
    mainRegulator.regulatorId: 1
    mainRegulator.title: "主调压器"
    mainRegulator.statusData: rosProxy.regulatorStatus1
    mainRegulator.controlMode: rosProxy.circuitStatus1 ? rosProxy.circuitStatus1.control_mode : 0

    // --- 辅调压器 ---
    auxRegulator.regulatorId: 2
    auxRegulator.title: "辅调压器"
    auxRegulator.statusData: rosProxy.regulatorStatus2
    auxRegulator.controlMode: rosProxy.circuitStatus2 ? rosProxy.circuitStatus2.control_mode : 0

    // --- 数据源常量 ---
    readonly property var testChannelModel: ["通道01", "通道02", "通道03", "通道04", "通道05", "通道06", "通道07", "通道08", "通道09", "通道10", "通道11", "通道12", "通道13", "通道14", "通道15", "通道16"]
    readonly property var refChannelModel: ["导体01", "导体02", "导体03", "护套01", "护套02", "护套03", "通道07", "通道08"]

    // --- [核心修改 1] 直接在这里绑定数据，而不是在 onCompleted 里写 JS ---
    circuit1.circuitId: 1
    circuit1.statusData: rosProxy.circuitStatus1
    circuit1.settingsData: rosProxy.qmlCircuitSettings1

    circuit2.circuitId: 2
    circuit2.statusData: rosProxy.circuitStatus2
    circuit2.settingsData: rosProxy.qmlCircuitSettings2

    currentSeries.axisYRight: axisYCurrent

    // --- 初始化 ---
    Component.onCompleted: {
        updateChannelModel();

        // --- [核心修改 2] 强制触发一次数据更新信号 ---
        // 既然 CircuitSettingsPage 需要这样做才能显示数据，这里也同样需要
        if (rosProxy) {
            rosProxy.qmlCircuitSettings1Changed();
            rosProxy.qmlCircuitSettings2Changed();
        }
    }

    // --- 逻辑控制 1: 下拉菜单联动 ---
    Connections {
        target: control.loopSelector
        function onCurrentIndexChanged() {
            updateChannelModel();
        }
    }

    function updateChannelModel() {
        var idx = control.loopSelector.currentIndex;
        if (idx === 0 || idx === 2) {
            control.channelSelector.model = testChannelModel;
        } else {
            control.channelSelector.model = refChannelModel;
        }
        control.channelSelector.currentIndex = 0;
    }

    // --- 逻辑控制 2: 数据采集与绘图 ---
    Connections {
        target: rosProxy
        function onCircuitStatus1Changed() { updateChartData(1); }
    }
    Connections {
        target: rosProxy
        function onCircuitStatus2Changed() { updateChartData(2); }
    }

    function updateChartData(sourceCircuitId) {
        var selectedIdx = control.loopSelector.currentIndex;
        var targetCircuitId = (selectedIdx <= 1) ? 1 : 2;
        if (targetCircuitId !== sourceCircuitId) return;

        var statusData = (targetCircuitId === 1) ? rosProxy.circuitStatus1 : rosProxy.circuitStatus2;
        if (!statusData) return;

        var isTestLoop = (selectedIdx % 2 === 0);
        var loopData = isTestLoop ? statusData.test_loop : statusData.ref_loop;

        // 如果 status 数据里没有 current，尝试去 settings 数据里找(通常图表是画实测值，这里保持原逻辑)
        var currentVal = (loopData && loopData.current !== undefined) ? loopData.current : 0;

        var channelIdx = control.channelSelector.currentIndex;
        var tempVal = 0.0;
        if (loopData && loopData.temperature_array && channelIdx >= 0 && channelIdx < loopData.temperature_array.length) {
            tempVal = loopData.temperature_array[channelIdx];
        }

        var now = new Date();
        control.tempSeries.append(now.getTime(), tempVal);
        control.currentSeries.append(now.getTime(), currentVal);
        updateAxisRange(now);
    }

    function updateAxisRange(nowDate) {
        var hours = control.timeRangeSelector.currentValue;
        if (!hours) hours = 2;
        var msRange = hours * 60 * 60 * 1000;
        var minDate = new Date(nowDate.getTime() - msRange);
        control.axisX.max = nowDate;
        control.axisX.min = minDate;
    }
}
