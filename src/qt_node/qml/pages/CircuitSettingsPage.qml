import QtQuick
import QtQuick.Controls
import qt_node 1.0

CircuitSettingsPageForm {
    id: page

    // 【核心】接收来自 Main.qml 的数据对象
    property var settingsData: null
    property var statusData: null

    // 存储从后端拉取到的所有电缆数据
    property var availableCables: []
    property bool isFetchingCables: false

    // ========================================================
    // 1. 辅助函数 (包含日期优化)
    // ========================================================
    function secToMin(sec) { return (sec / 60).toFixed(0); }
    function minToSec(minStr) { return parseInt(minStr) * 60; }

    function secToTimeStr(sec) {
        var h = Math.floor(sec / 3600).toString().padStart(2, '0');
        var m = Math.floor((sec % 3600) / 60).toString().padStart(2, '0');
        return h + ":" + m;
    }

    function secToHourPart(sec) { return Math.floor(sec / 3600).toString().padStart(2, '0'); }
    function secToMinPart(sec) { return Math.floor((sec % 3600) / 60).toString().padStart(2, '0'); }

    function dateToString(dt) {
        if(dt) return Qt.formatDateTime(dt, "yyyy-MM-dd");
        else return Qt.formatDateTime(new Date(), "yyyy-MM-dd");
    }

    function normalizeDateStr(str) {
        if (!str) return "";
        var cleanStr = str.replace(/[\/\.]/g, "-");
        var parts = cleanStr.split("-");
        if (parts.length === 3) {
            var y = parts[0];
            var m = parts[1].toString().padStart(2, '0');
            var d = parts[2].toString().padStart(2, '0');
            return y + "-" + m + "-" + d;
        }
        return str;
    }

    function stringToDate(str) {
        var normalized = normalizeDateStr(str);
        return Date.fromLocaleString(Qt.locale(), normalized, "yyyy-MM-dd");
    }

    function setLoopEnableState(loopType, enableState) {
        if (!settingsData) return;

        var data = settingsData;
        if (loopType === "test") {
            data.test_loop.enabled = enableState;
        } else if (loopType === "ref") {
            data.ref_loop.enabled = enableState;
        }

        rosProxy.setCircuitSettings(circuitId, data);
    }

    // ========================================================
    // 2. 数据获取与下拉菜单联动
    // ========================================================

    // 获取所有电缆用于填充下拉菜单 (查询空字符串, 页码1, 1000条, 按名称正序排列)
    function fetchAllCables() {
        isFetchingCables = true;
        rosProxy.listCables("", 1, 1000, 1, true);
    }

    Connections {
        target: rosProxy
        function onCablesListed(result) {
            // 只捕获此页面触发的全局查询请求
            if (result.success && page.isFetchingCables) {
                page.availableCables = result.cables;
                cableComboBox.model = page.availableCables;
                syncGeneralInputs(); // 下拉菜单加载完毕后，尝试选中系统中正在使用的电缆
                page.isFetchingCables = false;
            }
        }
    }

    // 当用户在下拉菜单中切换选中项时，实时更新下方的只读面板 (拼接单位)
    Connections {
        target: cableComboBox
        function onCurrentIndexChanged() {
            var idx = cableComboBox.currentIndex;
            if (idx >= 0 && idx < availableCables.length) {
                var c = availableCables[idx];
                lblCoreDia.text = c.core_diameter.toFixed(2) + " mm";
                lblCoreMat.text = c.core_material;
                lblInsThick.text = c.insulation_thickness.toFixed(2) + " mm";
                lblInsMat.text = c.insulation_material;
                lblVoltage.text = Math.round(c.voltage_grade / 1000) + " kV";
                lblFormat.text = c.system_format === 1 ? "交流" : "直流";
                lblNotes.text = c.notes;
            } else {
                lblCoreDia.text = "-";
                lblCoreMat.text = "-";
                lblInsThick.text = "-";
                lblInsMat.text = "-";
                lblVoltage.text = "-";
                lblFormat.text = "-";
                lblNotes.text = "-";
            }
        }
    }


    // ========================================================
    // 3. 数据同步逻辑
    // ========================================================

    function syncTestInputs() {
        if (!settingsData) return;
        var data = settingsData;
        var test = data.test_loop;

        testStrategyCombo.currentIndex = test.auto_strategy === 2 ? 1 : 0;
        testStartCurrent.settingValue = test.start_current_a;
        testMaxCurrent.settingValue = test.max_current_a;
        testChangePercent.settingValue = test.current_change_range_percent;
        testCtRatio.settingValue = test.ct_ratio;

        testStartDate.settingValue = dateToString(test.start_date);
        testCycleCount.settingValue = test.cycle_count;

        testHeatInputHour.text = secToHourPart(test.heating_start_time_sec);
        testHeatInputMin.text = secToMinPart(test.heating_start_time_sec);
        testHeatingDuration.settingValue = secToMin(test.heating_duration_sec);
    }

    function syncRefInputs() {
        if (!settingsData) return;
        var data = settingsData;
        var ref = data.ref_loop;

        refStrategyCombo.currentIndex = ref.auto_strategy === 2 ? 1 : 0;
        refStartCurrent.settingValue = ref.start_current_a;
        refMaxCurrent.settingValue = ref.max_current_a;
        refChangePercent.settingValue = ref.current_change_range_percent;
        refCtRatio.settingValue = ref.ct_ratio;

        refStartDate.settingValue = dateToString(ref.start_date);
        refCycleCount.settingValue = ref.cycle_count;

        refHeatInputHour.text = secToHourPart(ref.heating_start_time_sec);
        refHeatInputMin.text = secToMinPart(ref.heating_start_time_sec);
        refHeatingDuration.settingValue = secToMin(ref.heating_duration_sec);
    }

    // 【核心】根据当前系统里生效的电缆名称，去列表中寻找并选中它 (这就是“还原”逻辑)
    function syncGeneralInputs() {
        if (!settingsData || availableCables.length === 0) return;

        // 🚨 修改点 1：将匹配依据由 name 改为 id，这是启动时唯一绝对可靠的标识符
        var appliedCableId = settingsData.sample_cable.id;

        var foundIdx = -1;
        for (var i = 0; i < availableCables.length; i++) {
            // 🚨 修改点 2：根据 id 匹配
            if (availableCables[i].id === appliedCableId) {
                foundIdx = i;
                break;
            }
        }

        if (foundIdx >= 0) {
            cableComboBox.currentIndex = foundIdx;
        } else {
            // 如果没找到对应的 ID（可能是该电缆在全局库中被永久删除了），默认选中第一条
            if (cableComboBox.currentIndex === -1 && availableCables.length > 0) {
                cableComboBox.currentIndex = 0;
            }
        }
    }

    function syncInputs() {
        if (!settingsData) return;
        syncTestInputs();
        syncRefInputs();
        syncGeneralInputs();
    }

    // 更新左侧反馈
    function refreshFeedback() {
        if (!settingsData) return;
        var data = settingsData;

        testStartStop.isRunning = data.test_loop.enabled;
        testStartCurrent.currentValue = data.test_loop.start_current_a;
        testMaxCurrent.currentValue = data.test_loop.max_current_a;
        testChangePercent.currentValue = data.test_loop.current_change_range_percent;
        testCtRatio.currentValue = data.test_loop.ct_ratio;
        testStartDate.currentValue = dateToString(data.test_loop.start_date);
        testCycleCount.currentValue = data.test_loop.cycle_count;
        testHeatFeedback.text = secToTimeStr(data.test_loop.heating_start_time_sec);
        testHeatingDuration.currentValue = secToMin(data.test_loop.heating_duration_sec);

        refStartStop.isRunning = data.ref_loop.enabled;
        refStartCurrent.currentValue = data.ref_loop.start_current_a;
        refMaxCurrent.currentValue = data.ref_loop.max_current_a;
        refChangePercent.currentValue = data.ref_loop.current_change_range_percent;
        refCtRatio.currentValue = data.ref_loop.ct_ratio;
        refStartDate.currentValue = dateToString(data.ref_loop.start_date);
        refCycleCount.currentValue = data.ref_loop.cycle_count;
        refHeatFeedback.text = secToTimeStr(data.ref_loop.heating_start_time_sec);
        refHeatingDuration.currentValue = secToMin(data.ref_loop.heating_duration_sec);
    }

    // ========================================================
    // 4. 事件绑定
    // ========================================================

    Connections {
        target: rosProxy
        function onQmlCircuitSettings1Changed() {
            if (page.circuitId === 1) refreshFeedback();
        }
        function onQmlCircuitSettings2Changed() {
            if (page.circuitId === 2) refreshFeedback();
        }
    }

    onVisibleChanged: {
        if (visible) {
            fetchAllCables(); // 每次切入页面，刷新线缆库
            syncInputs();
            refreshFeedback();
        }
    }

    Component.onCompleted: {
        if (circuitId === 1) rosProxy.qmlCircuitSettings1Changed();
        else rosProxy.qmlCircuitSettings2Changed();

        syncInputs();
    }

    testStartStop.onStartClicked: setLoopEnableState("test", true)
    testStartStop.onStopClicked: setLoopEnableState("test", false)
    refStartStop.onStartClicked: setLoopEnableState("ref", true)
    refStartStop.onStopClicked: setLoopEnableState("ref", false)

    function applySettings(loopType) {
        if (!settingsData) return;
        var data = settingsData;
        var loop = (loopType === "test") ? data.test_loop : data.ref_loop;

        var strategyCombo = (loopType === "test") ? testStrategyCombo : refStrategyCombo;
        var startCur = (loopType === "test") ? testStartCurrent : refStartCurrent;
        var maxCur = (loopType === "test") ? testMaxCurrent : refMaxCurrent;
        var chgPct = (loopType === "test") ? testChangePercent : refChangePercent;
        var ctRatio = (loopType === "test") ? testCtRatio : refCtRatio
        var startDate = (loopType === "test") ? testStartDate : refStartDate;
        var cycleCnt = (loopType === "test") ? testCycleCount : refCycleCount;
        var hInput = (loopType === "test") ? testHeatInputHour : refHeatInputHour;
        var mInput = (loopType === "test") ? testHeatInputMin : refHeatInputMin;
        var durInput = (loopType === "test") ? testHeatingDuration : refHeatingDuration;

        loop.auto_strategy = (strategyCombo.currentIndex === 1) ? 2 : 1;
        loop.start_current_a = parseInt(startCur.settingValue);
        loop.max_current_a = parseInt(maxCur.settingValue);
        loop.current_change_range_percent = parseInt(chgPct.settingValue);
        loop.ct_ratio = parseInt(ctRatio.settingValue);

        var cleanDateStr = normalizeDateStr(startDate.settingValue);
        startDate.settingValue = cleanDateStr;
        var dateObj = stringToDate(cleanDateStr);
        if(!isNaN(dateObj.getTime())) loop.start_date = dateObj;

        loop.cycle_count = parseInt(cycleCnt.settingValue);

        var h = parseInt(hInput.text);
        var m = parseInt(mInput.text);
        loop.heating_start_time_sec = h * 3600 + m * 60;

        loop.heating_duration_sec = minToSec(durInput.settingValue);

        rosProxy.setCircuitSettings(circuitId, data);
    }

    applyLeftBtn.onClicked: applySettings("test")
    applyMidBtn.onClicked: applySettings("ref")

    // 【应用电缆设定】
    applyRightBtn.onClicked: {
        if (!settingsData) return;
        var idx = cableComboBox.currentIndex;
        if (idx < 0 || idx >= availableCables.length) {
            messagePopup.isError = true;
            messagePopup.message = "请先在下拉菜单中选择一个有效的电缆！";
            messagePopup.open();
            return;
        }

        var c = availableCables[idx];
        var data = settingsData;
        var cable = data.sample_cable;

        // 从后端拉下来的字典对象直接覆盖进去
        cable.id = c.id;
        cable.name = c.name;
        cable.core_diameter = c.core_diameter;
        cable.core_material = c.core_material;
        cable.insulation_thickness = c.insulation_thickness;
        cable.insulation_material = c.insulation_material;
        cable.voltage_grade = c.voltage_grade; // 已经在后端查出来是 V 单位
        cable.system_format = c.system_format;
        cable.notes = c.notes;

        rosProxy.setCircuitSettings(circuitId, data);
    }

    restoreLeftBtn.onClicked: syncTestInputs()
    restoreMidBtn.onClicked: syncRefInputs()
    restoreRightBtn.onClicked: syncGeneralInputs() // 【还原电缆设定】

    Connections {
        target: rosProxy
        function onSettingsUpdateResult(srv, success, msg) {
            if(!success) {
                messagePopup.isError = true;
                messagePopup.message = msg;
                messagePopup.open();
            }
        }
    }
}
