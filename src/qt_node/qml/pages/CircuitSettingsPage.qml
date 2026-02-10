import QtQuick
import QtQuick.Controls
import qt_node 1.0

CircuitSettingsPageForm {
    id: page

    // 默认1，外部调用时在 Main.qml 中设为 2
    // property int circuitId: 1 // 【注意】这一行必须删除，使用 Form 中定义的 circuitId

    // 【核心】接收来自 Main.qml 的数据对象
    property var settingsData: null
    property var statusData: null

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

    // 【智能日期清洗函数】
    function normalizeDateStr(str) {
        if (!str) return "";
        // 1. 统一分隔符：把 / 和 . 都替换成 -
        var cleanStr = str.replace(/[\/\.]/g, "-");
        // 2. 拆分
        var parts = cleanStr.split("-");
        // 3. 如果是标准的3段式 (年-月-日)
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

    // 用启停控制逻辑
    function setLoopEnableState(loopType, enableState) {
        if (!settingsData) return;

        // 1. 深拷贝当前数据（或者是直接引用，取决于你的Proxy实现，通常直接修改var对象即可）
        var data = settingsData;

        // 2. 修改对应的 enable 字段
        if (loopType === "test") {
            data.test_loop.enabled = enableState;
        } else if (loopType === "ref") {
            data.ref_loop.enabled = enableState;
        }

        // 3. 将修改后的整个配置结构体回写给后端
        // 后端收到后，应当根据 enabled 的变化去驱动底层硬件
        rosProxy.setCircuitSettings(circuitId, data);
    }

    // ========================================================
    // 2. 数据同步 (拆分逻辑以支持还原功能)
    // ========================================================

    // 同步左侧 (试验回路)
    function syncTestInputs() {
        if (!settingsData) return;
        var data = settingsData;
        var test = data.test_loop;

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

    // 同步中间 (模拟回路)
    function syncRefInputs() {
        if (!settingsData) return;
        var data = settingsData;
        var ref = data.ref_loop;

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

    // 同步右侧 (通用/被试品)
    function syncGeneralInputs() {
        if (!settingsData) return;
        var data = settingsData;

        // 右上角下拉
        refSourceCombo.currentIndex = data.curr_mode_use_ref ? 1 : 0;

        // 右下角被试品
        var sample = data.sample_params;
        sampleType.settingValue = sample.cable_type;
        sampleSpec.settingValue = sample.cable_spec;
        sampleInsMaterial.settingValue = sample.insulation_material;

        // 【修改】绝缘厚度：保留两位小数转为字符串
        // 如果后端传回 0，则显示 "0.00"
        sampleInsThick.settingValue = sample.insulation_thickness ? sample.insulation_thickness.toFixed(2) : "0.00";
    }

    // 总同步入口
    function syncInputs() {
        if (!settingsData) return;
        syncTestInputs();
        syncRefInputs();
        syncGeneralInputs();
    }

    // 刷新反馈显示 (实心框)
    function refreshFeedback() {
        if (!settingsData) return;
        var data = settingsData;

        // --- Test Loop Feedback ---
        testStartStop.isRunning = data.test_loop.enabled;
        testStartCurrent.currentValue = data.test_loop.start_current_a;
        testMaxCurrent.currentValue = data.test_loop.max_current_a;
        testChangePercent.currentValue = data.test_loop.current_change_range_percent;
        testCtRatio.currentValue = data.test_loop.ct_ratio;

        testStartDate.currentValue = dateToString(data.test_loop.start_date);
        testCycleCount.currentValue = data.test_loop.cycle_count;
        testHeatFeedback.text = secToTimeStr(data.test_loop.heating_start_time_sec);
        testHeatingDuration.currentValue = secToMin(data.test_loop.heating_duration_sec);

        // --- Ref Loop Feedback ---
        refStartStop.isRunning = data.ref_loop.enabled;
        refStartCurrent.currentValue = data.ref_loop.start_current_a;
        refMaxCurrent.currentValue = data.ref_loop.max_current_a;
        refChangePercent.currentValue = data.ref_loop.current_change_range_percent;
        refCtRatio.currentValue = data.ref_loop.ct_ratio;

        refStartDate.currentValue = dateToString(data.ref_loop.start_date);
        refCycleCount.currentValue = data.ref_loop.cycle_count;
        refHeatFeedback.text = secToTimeStr(data.ref_loop.heating_start_time_sec);
        refHeatingDuration.currentValue = secToMin(data.ref_loop.heating_duration_sec);

        // --- Ref Source Feedback ---
        refSourceDisplay.text = data.curr_mode_use_ref ? ("模拟回路" + circuitId) : ("试验回路" + circuitId);

        // --- Sample Feedback ---
        sampleType.currentValue = data.sample_params.cable_type;

        // 绝缘厚度反馈：同样保留两位小数显示
        // SettingInput 组件通常有 currentValue 属性用于显示反馈值
        sampleInsThick.currentValue = data.sample_params.insulation_thickness ? data.sample_params.insulation_thickness.toFixed(2) : "0.00";
    }

    // --- 信号监听 ---

    // 监听 Settings 变化 (反馈值更新)
    Connections {
        target: rosProxy
        function onQmlCircuitSettings1Changed() {
            if (page.circuitId === 1) refreshFeedback();
        }
        function onQmlCircuitSettings2Changed() {
            if (page.circuitId === 2) refreshFeedback();
        }
    }

    // 【新增】页面可见性变化时自动反填
    onVisibleChanged: {
        if (visible) {
            syncInputs();
            refreshFeedback();
        }
    }

    // 初始化
    Component.onCompleted: {
        // 初始刷新
        if (circuitId === 1) rosProxy.qmlCircuitSettings1Changed();
        else rosProxy.qmlCircuitSettings2Changed();

        syncInputs();
    }

    // ========================================================
    // 3. 按钮逻辑 & 下拉逻辑
    // ========================================================

    testStartStop.onStartClicked: setLoopEnableState("test", true)
    testStartStop.onStopClicked: setLoopEnableState("test", false)
    refStartStop.onStartClicked: setLoopEnableState("ref", true)
    refStartStop.onStopClicked: setLoopEnableState("ref", false)

    Connections {
        target: page.refSourceCombo
        function onActivated(index) {
            var useRef = (index === 1);
            rosProxy.setCircuitReferenceSource(circuitId, useRef);
        }
    }

    // 通用应用函数
    function applySettings(loopType) {
        if (!settingsData) return;
        var data = settingsData;
        var loop = (loopType === "test") ? data.test_loop : data.ref_loop;

        var startCur = (loopType === "test") ? testStartCurrent : refStartCurrent;
        var maxCur = (loopType === "test") ? testMaxCurrent : refMaxCurrent;
        var chgPct = (loopType === "test") ? testChangePercent : refChangePercent;
        var ctRatio = (loopType === "test") ? testCtRatio : refCtRatio
        var startDate = (loopType === "test") ? testStartDate : refStartDate;
        var cycleCnt = (loopType === "test") ? testCycleCount : refCycleCount;
        var hInput = (loopType === "test") ? testHeatInputHour : refHeatInputHour;
        var mInput = (loopType === "test") ? testHeatInputMin : refHeatInputMin;
        var durInput = (loopType === "test") ? testHeatingDuration : refHeatingDuration;

        loop.start_current_a = parseInt(startCur.settingValue);
        loop.max_current_a = parseInt(maxCur.settingValue);
        loop.current_change_range_percent = parseInt(chgPct.settingValue);
        loop.ct_ratio = parseInt(ctRatio.settingValue);

        // 日期清洗与应用
        var cleanDateStr = normalizeDateStr(startDate.settingValue);
        startDate.settingValue = cleanDateStr; // 反填清洗后的值
        var dateObj = stringToDate(cleanDateStr);
        if(!isNaN(dateObj.getTime())) loop.start_date = dateObj;

        loop.cycle_count = parseInt(cycleCnt.settingValue);

        var h = parseInt(hInput.text);
        var m = parseInt(mInput.text);
        loop.heating_start_time_sec = h * 3600 + m * 60;

        loop.heating_duration_sec = minToSec(durInput.settingValue);

        rosProxy.setCircuitSettings(circuitId, data);
    }

    // 绑定应用按钮
    applyLeftBtn.onClicked: applySettings("test")
    applyMidBtn.onClicked: applySettings("ref")

    // 绑定被试品应用按钮
    applyRightBtn.onClicked: {
        if (!settingsData) return;
        var data = settingsData;
        var sample = data.sample_params;

        sample.cable_type = sampleType.settingValue;
        sample.cable_spec = sampleSpec.settingValue;
        sample.insulation_material = sampleInsMaterial.settingValue;
        // 验证器保证了格式正确，但 parseFloat 是为了确保类型安全
        var thickVal = parseFloat(sampleInsThick.settingValue);
        sample.insulation_thickness = isNaN(thickVal) ? 0.0 : thickVal;

        rosProxy.setCircuitSettings(circuitId, data);
    }

    // 绑定还原按钮
    restoreLeftBtn.onClicked: syncTestInputs()
    restoreMidBtn.onClicked: syncRefInputs()
    restoreRightBtn.onClicked: syncGeneralInputs()

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
