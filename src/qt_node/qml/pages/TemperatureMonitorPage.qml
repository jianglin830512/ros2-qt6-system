import QtQuick
import qt_node 1.0
import "../components"

TemperatureMonitorPageForm {
    id: control

    mainRegulator.regulatorId: 1
    mainRegulator.title: "主调压器"
    mainRegulator.statusData: rosProxy.regulatorStatus1
    mainRegulator.controlMode: (rosProxy.circuitStatus1 && rosProxy.circuitStatus1.control_mode !== undefined) ? rosProxy.circuitStatus1.control_mode : 0

    auxRegulator.regulatorId: 2
    auxRegulator.title: "辅调压器"
    auxRegulator.statusData: rosProxy.regulatorStatus2
    auxRegulator.controlMode: (rosProxy.circuitStatus2 && rosProxy.circuitStatus2.control_mode !== undefined) ? rosProxy.circuitStatus2.control_mode : 0

    circuit1.circuitId: 1
    circuit1.statusData: rosProxy.circuitStatus1
    circuit1.settingsData: rosProxy.qmlCircuitSettings1
    circuit1.mainRegulatorClosed: rosProxy.regulatorStatus1 ? rosProxy.regulatorStatus1.breaker_closed_switch_ack : false
    circuit1.auxRegulatorClosed: rosProxy.regulatorStatus2 ? rosProxy.regulatorStatus2.breaker_closed_switch_ack : false

    circuit2.circuitId: 2
    circuit2.statusData: rosProxy.circuitStatus2
    circuit2.settingsData: rosProxy.qmlCircuitSettings2
    circuit2.mainRegulatorClosed: rosProxy.regulatorStatus1 ? rosProxy.regulatorStatus1.breaker_closed_switch_ack : false
    circuit2.auxRegulatorClosed: rosProxy.regulatorStatus2 ? rosProxy.regulatorStatus2.breaker_closed_switch_ack : false

    currentSeries.axisYRight: axisYCurrent

    property var historyCache: ({})
    property var lastSaveTime: ({ 1: 0, 2: 0 })

    // ==========================================
    // 动态生成温度通道下拉列表
    // ==========================================
    function updateChannelModel() {
        if (typeof rosProxy === "undefined" || !rosProxy || !rosProxy.systemStatus) return;

        var monitorType = rosProxy.systemStatus.temp_monitor_type;
        var numChannels = (monitorType === 1) ? 32 : 40;

        var cId = (control.loopSelector.currentIndex <= 1) ? 1 : 2;
        var startCard = (cId === 1) ? 1 : ((monitorType === 1) ? 5 : 6);

        var arr = [];
        for (var i = 0; i < numChannels; i++) {
           var r = Math.floor(i / 8) + startCard;
           var c = (i % 8) + 1;
           arr.push("卡" + r + "通" + c);
        }

        var oldIdx = control.channelSelector.currentIndex;
        control.channelSelector.model = arr;

        // 保持索引防止切换时越界
        if (oldIdx >= 0 && oldIdx < arr.length) {
            control.channelSelector.currentIndex = oldIdx;
        } else {
            control.channelSelector.currentIndex = 0;
        }
    }

    function getCurrentKeys() {
        var idx = control.loopSelector.currentIndex;
        var cId = (idx <= 1) ? 1 : 2;
        var isTest = (idx % 2 === 0);
        var tIdx = control.channelSelector.currentIndex;
        if(tIdx < 0) tIdx = 0; // 防御性越界处理

        var prefix = "c" + cId;
        return { curKey: prefix + "_" + (isTest ? "test" : "ref") + "_cur", tempKey: prefix + "_t" + tIdx };
    }

    Component.onCompleted: {
        updateChannelModel();
        if (typeof rosProxy !== "undefined" && rosProxy) {
            rosProxy.qmlCircuitSettings1Changed();
            rosProxy.qmlCircuitSettings2Changed();
        }
    }

    // 监控系统硬件状态类型的变化以动态调整通道选项数量
    Connections {
        target: typeof rosProxy !== "undefined" ? rosProxy : null
        function onSystemStatusChanged() {
            var monitorType = rosProxy.systemStatus.temp_monitor_type;
            var expectedCount = (monitorType === 1) ? 32 : 40;
            if (control.channelSelector.model && control.channelSelector.model.length !== expectedCount) {
                updateChannelModel();
            }
        }
    }

    Connections {
        target: control.loopSelector
        function onCurrentIndexChanged() {
            updateChannelModel();
            renderCurrentChart();
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

    function getLatestTimeMs() {
        var keys = getCurrentKeys();
        var curArray = historyCache[keys.curKey] || [];
        var tempArray = historyCache[keys.tempKey] || [];

        var latest = new Date().getTime();
        if (tempArray.length > 0) latest = tempArray[tempArray.length - 1].x;
        if (curArray.length > 0 && curArray[curArray.length - 1].x > latest) latest = curArray[curArray.length - 1].x;
        return latest;
    }

    function renderCurrentChart() {
        if (!control.tempSeries || !control.currentSeries) return;

        var keys = getCurrentKeys();
        var curArray = historyCache[keys.curKey] || [];
        var tempArray = historyCache[keys.tempKey] || [];

        control.tempSeries.clear();
        control.currentSeries.clear();

        for (var i = 0; i < tempArray.length; i++) control.tempSeries.append(tempArray[i].x, tempArray[i].y);
        for (var j = 0; j < curArray.length; j++) control.currentSeries.append(curArray[j].x, curArray[j].y);

        updateAxisRange(getLatestTimeMs());
    }

    Connections { target: typeof rosProxy !== "undefined" ? rosProxy : null; function onCircuitStatus1Changed() { processIncomingData(1, rosProxy.circuitStatus1); } }
    Connections { target: typeof rosProxy !== "undefined" ? rosProxy : null; function onCircuitStatus2Changed() { processIncomingData(2, rosProxy.circuitStatus2); } }

    function processIncomingData(cId, statusData) {
        if (!statusData || statusData.circuit_id === 0) return;

        var isValidData = false;
        var circuitTemps = statusData.temperature_array || [];
        for (var k = 0; k < circuitTemps.length; k++) {
            if (circuitTemps[k] !== 0) { isValidData = true; break; }
        }
        if (!isValidData) return;

        var nowMs = new Date().getTime();
        if (nowMs - lastSaveTime[cId] < 10000) return;
        lastSaveTime[cId] = nowMs;

        function saveAndThinData(key, val) {
            if (!historyCache[key]) historyCache[key] = [];
            var arr = historyCache[key];

            arr.push({x: nowMs, y: val});

            var maxCacheMs = 60 * 60 * 1000;
            var thinThresholdMs = 10 * 60 * 1000;

            while (arr.length > 0 && nowMs - arr[0].x > maxCacheMs) arr.shift();

            if (arr.length > 1) {
                var lastKeptTime = arr[0].x;
                for (var i = 1; i < arr.length; i++) {
                    var ptTime = arr[i].x;
                    if (nowMs - ptTime <= thinThresholdMs) break;
                    if (ptTime - lastKeptTime < 30000) { arr.splice(i, 1); i--; } else { lastKeptTime = ptTime; }
                }
            }
        }

        if (statusData.test_loop) saveAndThinData("c" + cId + "_test_cur", statusData.test_loop.current || 0);
        if (statusData.ref_loop) saveAndThinData("c" + cId + "_ref_cur", statusData.ref_loop.current || 0);

        for (var i = 0; i < circuitTemps.length; i++) {
            saveAndThinData("c" + cId + "_t" + i, circuitTemps[i]);
        }

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

    function updateAxisRange(latestMs) {
        var idx = control.timeRangeSelector.currentIndex;
        var modelArr = control.timeRangeSelector.model;
        var mins = 10;
        if (modelArr && idx >= 0 && idx < modelArr.length) mins = modelArr[idx].value;
        var msRange = mins * 60 * 1000;
        control.axisX.max = new Date(latestMs);
        control.axisX.min = new Date(latestMs - msRange);
    }

    function limitChartSeriesPoints(series, nowMs) {
        if (!series || series.count === 0) return;
        var idx = control.timeRangeSelector.currentIndex;
        var modelArr = control.timeRangeSelector.model;
        var mins = 10;
        if (modelArr && idx >= 0 && idx < modelArr.length) mins = modelArr[idx].value;
        var msRange = mins * 60 * 1000;
        var thresholdTime = nowMs - msRange - 120000;
        while(series.count > 0 && series.at(0).x < thresholdTime) series.remove(0);
    }

    Connections{ target: control.btnManualMode; function onSendCommand() { if (typeof rosProxy !== "undefined" && rosProxy.qmlSystemSettings) { var sysData = rosProxy.qmlSystemSettings; sysData.auto_on = false; rosProxy.setSystemSettings(sysData); } } }
    Connections{ target: control.btnAutoMode; function onSendCommand() { if (typeof rosProxy !== "undefined" && rosProxy.qmlSystemSettings) { var sysData = rosProxy.qmlSystemSettings; sysData.auto_on = true; rosProxy.setSystemSettings(sysData); } } }
}
