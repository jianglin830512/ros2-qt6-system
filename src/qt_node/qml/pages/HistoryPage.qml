import QtQuick
import QtCharts
import qt_node 1.0
import "../components"

HistoryPageForm {
    id: page

    // ==========================================================
    // 1. 动态生成数据列 (主辅调压器 + 回路1/2 试验/参考 各种温度电流)
    // ==========================================================
    property var availableColumns: []

    function generateColumns() {
        var cols = [];

        // --- 提取公共逻辑，减少重复代码 ---
        function addCircuitColumns(cid, titlePrefix) {
            // 调压器
            cols.push({ key: cid + "|regulator_voltage", label: titlePrefix + "调压器 电压(V)" });
            cols.push({ key: cid + "|regulator_current", label: titlePrefix + "调压器 电流(A)" });

            // 回路电流
            cols.push({ key: cid + "|test_loop_current", label: "回路" + cid + " 试验支路电流(A)" });
            cols.push({ key: cid + "|ref_loop_current", label: "回路" + cid + " 参考支路电流(A)" });

            // 试验支路温度 1~16
            for (var i = 1; i <= 16; i++) {
                var numStr1 = (i < 10 ? "0" : "") + i;
                cols.push({ key: cid + "|test_loop_temp" + numStr1, label: "回路" + cid + " 试验温度 " + numStr1 });
            }

            // 参考支路温度 1~8
            for (var j = 1; j <= 8; j++) {
                var numStr2 = (j < 10 ? "0" : "") + j;
                cols.push({ key: cid + "|ref_loop_temp" + numStr2, label: "回路" + cid + " 参考温度 " + numStr2 });
            }
        }

        addCircuitColumns(1, "主"); // 回路1 / 主调压器
        addCircuitColumns(2, "辅"); // 回路2 / 辅调压器

        return cols;
    }

    Component.onCompleted: {
        availableColumns = generateColumns();
        colRepeater.model = availableColumns;
    }

    // ==========================================================
    // 2. 勾选限制逻辑 (Max 10)
    // ==========================================================
    property int checkedCount: 0

    Connections {
        target: colRepeater
        function onItemAdded(index, item) {
            item.onCheckedChanged.connect(function() {
                var cnt = 0;
                for (var i = 0; i < colRepeater.count; i++) {
                    if (colRepeater.itemAt(i).checked) cnt++;
                }
                page.checkedCount = cnt;

                // 如果超过10个，强制取消当前的勾选
                if (page.checkedCount > 10 && item.checked) {
                    item.checked = false;
                    messagePopup.message = "最多只能同时查看 10 条曲线！";
                    messagePopup.isError = false;
                    messagePopup.open();
                }
            });
        }
    }

    // ==========================================================
    // 3. 点击查询
    // ==========================================================
    queryBtn.onClicked: {
        var selectedCols = [];
        for (var i = 0; i < colRepeater.count; i++) {
            var item = colRepeater.itemAt(i);
            if (item.checked) {
                selectedCols.push(availableColumns[i].key);
            }
        }

        if (selectedCols.length === 0) {
            messagePopup.message = "请在左侧至少勾选一项数据！";
            messagePopup.isError = true;
            messagePopup.open();
            return;
        }

        var dateStr = dateInput.text;
        var timeStr = timeInput.text;
        var span = parseInt(spanCombo.currentValue);

        chartView.removeAllSeries();
        rosProxy.queryHistory(dateStr, timeStr, span, selectedCols);
    }

    // ==========================================================
    // 4. 数据接收与绘图
    // ==========================================================
    Connections {
        target: rosProxy

        function onHistoryQueryError(msg) {
            messagePopup.message = "查询失败: " + msg;
            messagePopup.isError = true;
            messagePopup.open();
        }

        function onHistoryDataReady(dataMap) {
            console.log("=== Received History Data in QML ===");

            // ... (时间范围和极值变量保持不变) ...
            var dateParts = dateInput.text.split("-");
            var timeParts = timeInput.text.split(":");
            var startDate = new Date(dateParts[0], dateParts[1]-1, dateParts[2], timeParts[0], timeParts[1], 0);
            var spanHours = parseInt(spanCombo.currentValue);
            var endDate = new Date(startDate.getTime() + spanHours * 3600 * 1000);

            axisX.min = startDate;
            axisX.max = endDate;

            var minTemp = 999999, maxTemp = -999999, hasTemp = false;
            var minVol  = 999999, maxVol  = -999999, hasVol  = false;
            var minCur  = 999999, maxCur  = -999999, hasCur  = false;

            var dataFound = false;

            // 3. 遍历数据生成曲线
            for (var colKey in dataMap) {
                var pointsArray = dataMap[colKey];
                if (!pointsArray || pointsArray.length === 0) continue;

                dataFound = true;

                var isCurrent = colKey.indexOf("current") !== -1;
                var isVoltage = colKey.indexOf("voltage") !== -1;

                var seriesName = getLabelByKey(colKey);
                var series = null;

                // --- 【核心修改】：不同类型绑定到不同的方向 ---
                if (isCurrent) {
                    // 电流：绑定到右侧。
                    // 创建时不传 Y 轴参数，随后通过 axisYRight 手动绑定
                    series = chartView.createSeries(ChartView.SeriesTypeLine, seriesName, axisX);
                    series.axisYRight = axisYCurrent;
                } else if (isVoltage) {
                    // 电压：绑定到左侧
                    series = chartView.createSeries(ChartView.SeriesTypeLine, seriesName, axisX, axisYVoltage);
                } else {
                    // 温度：绑定到左侧
                    series = chartView.createSeries(ChartView.SeriesTypeLine, seriesName, axisX, axisYTemp);
                }

                series.width = 2;

                // 将数据填充到曲线并统计极值
                for (var i = 0; i < pointsArray.length; i++) {
                    var pt = pointsArray[i];
                    series.append(pt.x, pt.y);

                    var val = pt.y;
                    if (isCurrent) {
                        hasCur = true;
                        if (val < minCur) minCur = val;
                        if (val > maxCur) maxCur = val;
                    } else if (isVoltage) {
                        hasVol = true;
                        if (val < minVol) minVol = val;
                        if (val > maxVol) maxVol = val;
                    } else { // 温度
                        hasTemp = true;
                        if (val < minTemp) minTemp = val;
                        if (val > maxTemp) maxTemp = val;
                    }
                }
            }

            // ... (下方判断 hasTemp, hasVol, hasCur 控制轴范围和可见性的逻辑保持不变) ...
            if (!dataFound) {
                axisYTemp.visible = false;
                axisYVoltage.visible = false;
                axisYCurrent.visible = false;
                messagePopup.message = "查询成功，但在设定的时间范围内没有数据。";
                messagePopup.isError = false;
                messagePopup.open();
                return;
            }

            // 更新独立 Y 轴的可见性和范围
            axisYTemp.visible = hasTemp;
            if (hasTemp) {
                var marginT = (maxTemp - minTemp) * 0.1;
                if (marginT === 0) marginT = 5;
                axisYTemp.min = Math.max(0, minTemp - marginT);
                axisYTemp.max = maxTemp + marginT;
            }

            axisYVoltage.visible = hasVol;
            if (hasVol) {
                var marginV = (maxVol - minVol) * 0.1;
                if (marginV === 0) marginV = 10;
                axisYVoltage.min = Math.max(0, minVol - marginV);
                axisYVoltage.max = maxVol + marginV;
            }

            axisYCurrent.visible = hasCur;
            if (hasCur) {
                var marginC = (maxCur - minCur) * 0.1;
                if (marginC === 0) marginC = 10;
                axisYCurrent.min = 0;
                axisYCurrent.max = maxCur + marginC;
            }
        }
    }

    function getLabelByKey(key) {
        for (var i=0; i<availableColumns.length; i++) {
            if (availableColumns[i].key === key) return availableColumns[i].label;
        }
        return key;
    }

    MessagePopup { id: messagePopup }
}
