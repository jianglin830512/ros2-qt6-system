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

    circuit1.circuitId: 1
    circuit1.statusData: rosProxy.circuitStatus1
    circuit1.settingsData: rosProxy.qmlCircuitSettings1

    circuit2.circuitId: 2
    circuit2.statusData: rosProxy.circuitStatus2
    circuit2.settingsData: rosProxy.qmlCircuitSettings2

    currentSeries.axisYRight: axisYCurrent

    // ==========================================
    // --- 核心：前端 1 小时降采样缓存机制 ---
    // ==========================================
    property var historyCache: ({})
    property var lastSaveTime: ({ 1: 0, 2: 0 })

    function getCurrentKeys() {
        var idx = control.loopSelector.currentIndex;
        var cId = (idx <= 1) ? 1 : 2;
        var isTest = (idx % 2 === 0);
        var tIdx = control.channelSelector.currentIndex;

        var prefix = "c" + cId + "_" + (isTest ? "test" : "ref");
        return { curKey: prefix + "_cur", tempKey: prefix + "_t" + tIdx };
    }

    Component.onCompleted: {
        updateChannelModel();
        if (rosProxy) {
            rosProxy.qmlCircuitSettings1Changed();
            rosProxy.qmlCircuitSettings2Changed();
        }
    }

    // ==========================================
    // --- 逻辑控制 1: 下拉菜单切换与高速重绘 ---
    // ==========================================
    Connections {
        target: control.loopSelector
        function onCurrentIndexChanged() {
            updateChannelModel();
        }
    }

    Connections {
        target: control.channelSelector
        function onCurrentIndexChanged() {
            renderCurrentChart();
        }
    }

    Connections {
        target: control.timeRangeSelector
        function onCurrentIndexChanged() {
            renderCurrentChart();
        }
    }

    function updateChannelModel() {
        var idx = control.loopSelector.currentIndex;
        var oldChannelIdx = control.channelSelector.currentIndex;

        if (idx === 0 || idx === 2) {
            control.channelSelector.model = testChannelModel;
        } else {
            control.channelSelector.model = refChannelModel;
        }

        // 修复潜在的信号未触发Bug：如果原先就是0，设为0不会触发 onCurrentIndexChanged，需要手动调一次
        if (oldChannelIdx === 0) {
            renderCurrentChart();
        } else {
            control.channelSelector.currentIndex = 0;
        }
    }

    // 获取当前缓存中最新点的时间（如果没有数据，取当前时间）
    function getLatestTimeMs() {
        var keys = getCurrentKeys();
        var curArray = historyCache[keys.curKey] || [];
        var tempArray = historyCache[keys.tempKey] || [];

        var latest = new Date().getTime();
        if (tempArray.length > 0) latest = tempArray[tempArray.length - 1].x;
        if (curArray.length > 0 && curArray[curArray.length - 1].x > latest) {
            latest = curArray[curArray.length - 1].x;
        }
        return latest;
    }

    // --- 使用 replace() 进行原子级高性能重绘 ---
    function renderCurrentChart() {
        if (!control.tempSeries || !control.currentSeries) return;

        var keys = getCurrentKeys();
        var curArray = historyCache[keys.curKey] || [];
        var tempArray = historyCache[keys.tempKey] || [];

        // 1. 清空当前图表线条
        control.tempSeries.clear();
        control.currentSeries.clear();

        // 2. 循环 append 写入缓存数据
        // 得益于我们的降采样机制，这里最多只有一百来个点，瞬间就能画完，不会卡顿
        for (var i = 0; i < tempArray.length; i++) {
            control.tempSeries.append(tempArray[i].x, tempArray[i].y);
        }

        for (var j = 0; j < curArray.length; j++) {
            control.currentSeries.append(curArray[j].x, curArray[j].y);
        }

        // 3. 刷新坐标轴
        updateAxisRange(getLatestTimeMs());
    }

    // ==========================================
    // --- 逻辑控制 2: 高频数据采集与动态抽稀 ---
    // ==========================================
    Connections {
        target: rosProxy
        function onCircuitStatus1Changed() { processIncomingData(1, rosProxy.circuitStatus1); }
    }
    Connections {
        target: rosProxy
        function onCircuitStatus2Changed() { processIncomingData(2, rosProxy.circuitStatus2); }
    }

    function processIncomingData(cId, statusData) {
        if (!statusData) return;

        var nowMs = new Date().getTime();

        // 限流阀门：全局强制每 10 秒才允许处理一次数据
        if (nowMs - lastSaveTime[cId] < 10000) {
            return;
        }
        lastSaveTime[cId] = nowMs;

        function saveAndThinData(key, val) {
            if (!historyCache[key]) historyCache[key] = [];
            var arr = historyCache[key];

            arr.push({x: nowMs, y: val});

            var maxCacheMs = 60 * 60 * 1000;      // 1小时
            var thinThresholdMs = 10 * 60 * 1000; // 10分钟

            // 删除 1 小时之外的数据
            while (arr.length > 0 && nowMs - arr[0].x > maxCacheMs) {
                arr.shift();
            }

            // 抽稀 10 分钟外的数据，保证点距 >= 30秒
            if (arr.length > 1) {
                var lastKeptTime = arr[0].x;
                for (var i = 1; i < arr.length; i++) {
                    var ptTime = arr[i].x;
                    if (nowMs - ptTime <= thinThresholdMs) {
                        break;
                    }
                    if (ptTime - lastKeptTime < 30000) {
                        arr.splice(i, 1);
                        i--;
                    } else {
                        lastKeptTime = ptTime;
                    }
                }
            }
        }

        // 1. 采集并降采样
        if (statusData.test_loop) {
            var prefixTest = "c" + cId + "_test";
            saveAndThinData(prefixTest + "_cur", statusData.test_loop.current || 0);
            var testTemps = statusData.test_loop.temperature_array || [];
            for (var i = 0; i < testTemps.length; i++) {
                saveAndThinData(prefixTest + "_t" + i, testTemps[i]);
            }
        }

        if (statusData.ref_loop) {
            var prefixRef = "c" + cId + "_ref";
            saveAndThinData(prefixRef + "_cur", statusData.ref_loop.current || 0);
            var refTemps = statusData.ref_loop.temperature_array || [];
            for (var j = 0; j < refTemps.length; j++) {
                saveAndThinData(prefixRef + "_t" + j, refTemps[j]);
            }
        }

        // 2. 局部图表追加
        var currentKeys = getCurrentKeys();
        var selectedCid = (control.loopSelector.currentIndex <= 1) ? 1 : 2;

        if (selectedCid === cId) {
            var currentArr = historyCache[currentKeys.curKey];
            var tempArr = historyCache[currentKeys.tempKey];

            if (currentArr && currentArr.length > 0) {
                var latestCur = currentArr[currentArr.length - 1];
                control.currentSeries.append(latestCur.x, latestCur.y);
            }
            if (tempArr && tempArr.length > 0) {
                var latestTemp = tempArr[tempArr.length - 1];
                control.tempSeries.append(latestTemp.x, latestTemp.y);
            }

            updateAxisRange(nowMs);

            limitChartSeriesPoints(control.tempSeries, nowMs);
            limitChartSeriesPoints(control.currentSeries, nowMs);
        }
    }

    // --- 图表轴与内存清理 ---

    function updateAxisRange(latestMs) {
        var idx = control.timeRangeSelector.currentIndex;
        var modelArr = control.timeRangeSelector.model;

        // 核心修复：直接读取 model 数组里面的精确数值，绕过 currentValue 的延迟
        var mins = 10;
        if (modelArr && idx >= 0 && idx < modelArr.length) {
            mins = modelArr[idx].value;
        }

        var msRange = mins * 60 * 1000;

        control.axisX.max = new Date(latestMs);
        control.axisX.min = new Date(latestMs - msRange);
    }

    function limitChartSeriesPoints(series, nowMs) {
        if (!series || series.count === 0) return;

        var idx = control.timeRangeSelector.currentIndex;
        var modelArr = control.timeRangeSelector.model;
        var mins = 10;
        if (modelArr && idx >= 0 && idx < modelArr.length) {
            mins = modelArr[idx].value;
        }

        var msRange = mins * 60 * 1000;
        var thresholdTime = nowMs - msRange - 120000;

        while(series.count > 0 && series.at(0).x < thresholdTime) {
            series.remove(0);
        }
    }
}
