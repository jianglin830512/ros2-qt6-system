import QtQuick
import QtCharts
import qt_node 1.0
import "../components"

HistoryPageForm {
    id: page

    // ==========================================================
    // 智能日期与时间清洗函数
    // ==========================================================
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

    function normalizeTimeStr(str) {
        if (!str) return "";
        // 允许用户不小心用点或者横杠代替冒号
        var cleanStr = str.replace(/[\/\.\-]/g, ":");
        var parts = cleanStr.split(":");
        if (parts.length === 2) {
            var h = parts[0].toString().padStart(2, '0');
            var m = parts[1].toString().padStart(2, '0');
            return h + ":" + m;
        }
        return str;
    }

    // ==========================================================
    // 监听 UI 层的输入完成事件 (回车或失去焦点)
    // ==========================================================
    Connections {
        target: page.dateInput
        function onEditingFinished() {
            page.dateInput.text = page.normalizeDateStr(page.dateInput.text);
        }
    }

    Connections {
        target: page.timeInput
        function onEditingFinished() {
            page.timeInput.text = page.normalizeTimeStr(page.timeInput.text);
        }
    }

    // ==========================================================
    // 1. 动态生成数据列 (主辅调压器 + 回路1/2 试验/参考 各种温度电流)
    // ==========================================================

    // 【修改】使用 ListModel 便于更新内部颜色状态
    ListModel {
        id: columnsModel
    }

    function generateColumns() {
        columnsModel.clear();
        function addCircuitColumns(cid) {
            columnsModel.append({ key: cid + "|regulator_1_voltage", label: "回路" + cid + " 调压器1 电压(V)", lineColor: "" });
            columnsModel.append({ key: cid + "|regulator_1_current", label: "回路" + cid + " 调压器1 电流(A)", lineColor: "" });
            columnsModel.append({ key: cid + "|test_loop_current",   label: "回路" + cid + " 试验支路电流(A)", lineColor: "" });

            columnsModel.append({ key: cid + "|regulator_2_voltage", label: "回路" + cid + " 调压器2 电压(V)", lineColor: "" });
            columnsModel.append({ key: cid + "|regulator_2_current", label: "回路" + cid + " 调压器2 电流(A)", lineColor: "" });
            columnsModel.append({ key: cid + "|ref_loop_current",    label: "回路" + cid + " 参考支路电流(A)", lineColor: "" });

            for (var i = 1; i <= 16; i++) {
                var numStr1 = (i < 10 ? "0" : "") + i;
                columnsModel.append({ key: cid + "|test_loop_temp" + numStr1, label: "回路" + cid + " 试验温度 " + numStr1, lineColor: "" });
            }

            for (var j = 1; j <= 16; j++) {
                var numStr2 = (j < 10 ? "0" : "") + j;
                columnsModel.append({ key: cid + "|ref_loop_temp" + numStr2, label: "回路" + cid + " 参考温度 " + numStr2, lineColor: "" });
            }
        }

        addCircuitColumns(1);
        addCircuitColumns(2);
    }

    Component.onCompleted: {
        generateColumns();
        page.colRepeater.model = columnsModel;
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
                } else if (!item.checked) {
                    // 当取消勾选时，清除该项之前分配过的历史颜色
                    columnsModel.setProperty(index, "lineColor", "");
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
                // 现在是从 ListModel 里面取 key
                selectedCols.push(columnsModel.get(i).key);
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

        // 我们通过遍历将现有的曲线清空数据并隐藏，这样可以保留所有坐标轴的命
        for (var idx = 0; idx < chartView.count; idx++) {
            var s = chartView.series(idx);
            if (s) {
                s.clear();
                s.visible = false;
            }
        }

        // 并在重新查询时清除掉左侧的所有旧颜色标记
        for (var m = 0; m < columnsModel.count; m++) {
            if (colRepeater.itemAt(m).checked) {
                columnsModel.setProperty(m, "lineColor", "");
            }
        }

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

            // 遍历数据生成或重用曲线
            for (var colKey in dataMap) {
                var pointsArray = dataMap[colKey];
                if (!pointsArray || pointsArray.length === 0) continue;

                dataFound = true;

                var isCurrent = colKey.indexOf("current") !== -1;
                var isVoltage = colKey.indexOf("voltage") !== -1;

                var seriesName = getLabelByKey(colKey);
                var series = null;

                // 1. 查找图表中是否已经存在同名的曲线 (复用逻辑)
                for (var sIdx = 0; sIdx < chartView.count; sIdx++) {
                    if (chartView.series(sIdx).name === seriesName) {
                        series = chartView.series(sIdx);
                        break;
                    }
                }

                // 2. 如果之前没有创建过这条曲线，则新建它并绑定对应的坐标轴
                if (!series) {
                    series = chartView.createSeries(ChartView.SeriesTypeLine, seriesName);

                    if (!series) {
                        console.error("Failed to create series for: " + seriesName);
                        continue;
                    }

                    series.axisX = axisX;

                    // 根据类型绑定 Y 轴
                    if (isCurrent) {
                        series.axisYRight = axisYCurrent;
                    } else if (isVoltage) {
                        series.axisY = axisYVoltage;
                    } else {
                        series.axisY = axisYTemp;
                    }

                    series.width = 2;
                }

                // 3. 唤醒并显示曲线
                series.visible = true;

                // 4. 将数据点填充到曲线中，并统计极值
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
                    } else {
                        hasTemp = true;
                        if (val < minTemp) minTemp = val;
                        if (val > maxTemp) maxTemp = val;
                    }
                }
            }

            // --- 更新坐标轴范围和可见性 ---
            if (!dataFound) {
                axisYTemp.visible = false;
                axisYVoltage.visible = false;
                axisYCurrent.visible = false;
                messagePopup.message = "查询成功，但在设定的时间范围内没有数据。";
                messagePopup.isError = false;
                messagePopup.open();
                return;
            }

            // 【修改点】强制所有 Y 轴从 0 开始
            axisYTemp.visible = hasTemp;
            axisYTemp.visible = hasTemp;
            if (hasTemp) {
                // 为了防止极端情况下计算出错，余量按跨度来计算
                var marginT = Math.max(5, (maxTemp - (-15)) * 0.1);
                axisYTemp.min = -15;  // 设定为 -15
                axisYTemp.max = maxTemp + marginT;
            }

            axisYVoltage.visible = hasVol;
            if (hasVol) {
                var marginV = maxVol * 0.1;
                if (marginV === 0) marginV = 10;
                axisYVoltage.min = 0;
                axisYVoltage.max = maxVol + marginV;
            }

            axisYCurrent.visible = hasCur;
            if (hasCur) {
                var marginC = maxCur * 0.1;
                if (marginC === 0) marginC = 10;
                axisYCurrent.min = 0;
                axisYCurrent.max = maxCur + marginC;
            }

            // --- 【核心精髓】将图表分配的曲线颜色反向注入到左侧勾选标签中 ---
            for (var q = 0; q < chartView.count; q++) {
                var visSeries = chartView.series(q);
                if (visSeries && visSeries.visible) {
                    for (var n = 0; n < columnsModel.count; n++) {
                        if (columnsModel.get(n).label === visSeries.name) {
                            // 使用 .toString() 提取实际 hex 颜色码写回 model
                            columnsModel.setProperty(n, "lineColor", visSeries.color.toString());
                            break;
                        }
                    }
                }
            }
        }
    }

    function getLabelByKey(key) {
        for (var i=0; i<columnsModel.count; i++) {
            if (columnsModel.get(i).key === key) return columnsModel.get(i).label;
        }
        return key;
    }

    MessagePopup { id: messagePopup }
}
