import QtQuick
import qt_node 1.0
import "../components"

TemperatureMonitorPageForm {
    id: control

    // --- 主调压器 ---
    mainRegulator.regulatorId: 1
    mainRegulator.title: "主调压器"
    mainRegulator.statusData: rosProxy.regulatorStatus1

    // --- 辅调压器 ---
    auxRegulator.regulatorId: 2
    auxRegulator.title: "辅调压器"
    auxRegulator.statusData: rosProxy.regulatorStatus2

    // --- 数据源常量 ---
    readonly property var testChannelModel: ["通道01", "通道02", "通道03", "通道04", "通道05", "通道06", "通道07", "通道08", "通道09", "通道10", "通道11", "通道12", "通道13", "通道14", "通道15", "通道16"]
    readonly property var refChannelModel: ["导体01", "导体02", "导体03", "护套01", "护套02", "护套03", "通道07", "通道08"]

    // --- 初始化与绑定 ---
    Component.onCompleted: {
        // 1. 初始化调压器和回路的基本绑定
        control.circuit1.circuitId = 1;
        control.circuit1.statusData = Qt.binding(function() { return rosProxy.circuitStatus1; });
        control.circuit1.settingsData = Qt.binding(function() { return rosProxy.qmlCircuitSettings1; });

        control.circuit2.circuitId = 2;
        control.circuit2.statusData = Qt.binding(function() { return rosProxy.circuitStatus2; });
        control.circuit2.settingsData = Qt.binding(function() { return rosProxy.qmlCircuitSettings2; });

        control.currentSeries.axisYRight = axisYCurrent;

        // 2. 初始化下拉菜单联动
        updateChannelModel();
    }

    // --- 逻辑控制 1: 下拉菜单联动 ---

    // 【修改点】使用 Connections 消除 Qt Creator 的误报
    Connections {
        target: control.loopSelector
        function onCurrentIndexChanged() {
            updateChannelModel();
        }
    }

    function updateChannelModel() {
        // 索引说明: 0=C1试, 1=C1模, 2=C2试, 3=C2模
        // 注意：这里使用 control.loopSelector 确保指向明确
        var idx = control.loopSelector.currentIndex;
        if (idx === 0 || idx === 2) {
            control.channelSelector.model = testChannelModel;
        } else {
            control.channelSelector.model = refChannelModel;
        }
        control.channelSelector.currentIndex = 0; // 重置选中第一个
    }

    // --- 逻辑控制 2: 数据采集与绘图 ---

    // 监听 ROS 数据更新 (只要有任一回路状态更新，就尝试刷新图表)
    Connections {
        target: rosProxy
        function onCircuitStatus1Changed() { updateChartData(1); }
    }
    Connections {
        target: rosProxy
        function onCircuitStatus2Changed() { updateChartData(2); }
    }

    function updateChartData(sourceCircuitId) {
        // 1. 确定用户当前想看哪个回路的数据
        // loopSelector index: 0->C1_Test, 1->C1_Ref, 2->C2_Test, 3->C2_Ref
        var selectedIdx = control.loopSelector.currentIndex;

        // 如果数据更新源 不是 用户选中的回路ID，则忽略 (避免重复绘图)
        var targetCircuitId = (selectedIdx <= 1) ? 1 : 2;
        if (targetCircuitId !== sourceCircuitId) return;

        // 2. 获取对应的数据对象
        var statusData = (targetCircuitId === 1) ? rosProxy.circuitStatus1 : rosProxy.circuitStatus2;
        if (!statusData) return;

        // 3. 获取对应的 LoopStatus (Test 或 Ref)
        // 偶数为 TestLoop, 奇数为 RefLoop
        var isTestLoop = (selectedIdx % 2 === 0);
        var loopData = isTestLoop ? statusData.test_loop : statusData.ref_loop;

        // 4. 获取数值
        // 电流
        var currentVal = loopData.current;

        // 温度 (根据 Channel 下拉框索引)
        var channelIdx = control.channelSelector.currentIndex;
        var tempVal = 0.0;

        // 安全访问数组
        if (loopData.temperature_array && channelIdx >= 0 && channelIdx < loopData.temperature_array.length) {
            tempVal = loopData.temperature_array[channelIdx];
        }

        // 5. 添加到图表
        var now = new Date();

        // 添加点
        control.tempSeries.append(now.getTime(), tempVal);
        control.currentSeries.append(now.getTime(), currentVal);

        // 6. 更新 X 轴时间范围
        updateAxisRange(now);
    }

    function updateAxisRange(nowDate) {
        var hours = control.timeRangeSelector.currentValue; // 获取选中的小时数 (2, 4, 8...)
        if (!hours) hours = 2; // 默认防错

        var msRange = hours * 60 * 60 * 1000;
        var minDate = new Date(nowDate.getTime() - msRange);

        control.axisX.max = nowDate;
        control.axisX.min = minDate;
    }
}
