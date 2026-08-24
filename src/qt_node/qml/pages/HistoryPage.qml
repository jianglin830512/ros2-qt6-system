import QtQuick
import QtCharts
import qt_node 1.0
import "../components"

HistoryPageForm {
    id: page

    // ==========================================================
    // 1. 动态生成数据列 (顺序：主压主流、辅压辅流、试流模流、试温、模温)
    // ==========================================================

    ListModel {
        id: columnsModel
    }

    function generateColumns() {
        columnsModel.clear();

        columnsModel.append({ key: "regulator_1_voltage", label: "主调压器电压(V)", lineColor: "" });
        columnsModel.append({ key: "regulator_1_current", label: "主调压器电流(A)", lineColor: "" });
        columnsModel.append({ key: "regulator_2_voltage", label: "辅调压器电压(V)", lineColor: "" });
        columnsModel.append({ key: "regulator_2_current", label: "辅调压器电流(A)", lineColor: "" });
        columnsModel.append({ key: "test_loop_current",   label: "试验回路电流(A)", lineColor: "" });
        columnsModel.append({ key: "ref_loop_current",    label: "模拟回路电流(A)", lineColor: "" });

        for (var i = 1; i <= 16; i++) {
            var numStr1 = (i < 10 ? "0" : "") + i;
            columnsModel.append({ key: "test_loop_temp" + numStr1, label: "试验回路温度" + numStr1 + "(℃)", lineColor: "" });
        }

        // 按照需求，模拟回路温度只需截取前 8 个
        for (var j = 1; j <= 8; j++) {
            var numStr2 = (j < 10 ? "0" : "") + j;
            columnsModel.append({ key: "ref_loop_temp" + numStr2, label: "模拟回路温度" + numStr2 + "(℃)", lineColor: "" });
        }
    }

    Component.onCompleted: {
        generateColumns();
        page.colRepeater.model = columnsModel;
    }

    // ==========================================================
    // 2. 勾选限制逻辑 (Max 5)
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

                // 如果超过5个，强制取消当前的勾选
                if (page.checkedCount > 5 && item.checked) {
                    item.checked = false;
                    messagePopup.message = "最多只能同时查看 5 条曲线！";
                    messagePopup.isError = false;
                    messagePopup.open();
                } else if (!item.checked) {
                    columnsModel.setProperty(index, "lineColor", "");
                }
            });
        }
    }

    // ==========================================================
    // 3. 点击查询
    // ==========================================================
    Connections {
        target: page.queryPanel
        function onQueryClicked() {
            var cid = page.queryPanel.circuitId + 1; // 0->1, 1->2
            var selectedCols = [];

            for (var i = 0; i < colRepeater.count; i++) {
                var item = colRepeater.itemAt(i);
                if (item.checked) {
                    // 后端期待类似 "1|test_loop_current" 以在收到数据后分离 ID 和列名
                    selectedCols.push(cid + "|" + columnsModel.get(i).key);
                }
            }

            if (selectedCols.length === 0) {
                messagePopup.message = "请在左侧至少勾选一项数据！";
                messagePopup.isError = true;
                messagePopup.open();
                return;
            }

            var dateStr = page.queryPanel.dateString;
            // 将天数转为小时
            var spanHours = parseInt(page.queryPanel.spanDays) * 24;

            // 清空旧数据
            for (var idx = 0; idx < chartView.count; idx++) {
                var s = chartView.series(idx);
                if (s) {
                    s.clear();
                    s.visible = false;
                }
            }

            for (var m = 0; m < columnsModel.count; m++) {
                if (colRepeater.itemAt(m).checked) {
                    columnsModel.setProperty(m, "lineColor", "");
                }
            }

            // 发起查询 (时间写死为 00:00:00)
            rosProxy.queryHistory(dateStr, "00:00", spanHours, selectedCols);
        }
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
            var dateParts = page.queryPanel.dateString.split("-");
            var startDate = new Date(dateParts[0], dateParts[1]-1, dateParts[2], 0, 0, 0);
            var spanHours = parseInt(page.queryPanel.spanDays) * 24;
            var endDate = new Date(startDate.getTime() + spanHours * 3600 * 1000);

            // 更新 X 轴范围
            axisX.min = startDate;
            axisX.max = endDate;

            var minTemp = 999999, maxTemp = -999999, hasTemp = false;
            var minVol  = 999999, maxVol  = -999999, hasVol  = false;
            var minCur  = 999999, maxCur  = -999999, hasCur  = false;
            var dataFound = false;

            for (var colKey in dataMap) {
                var pointsArray = dataMap[colKey];
                if (!pointsArray || pointsArray.length === 0) continue;

                dataFound = true;
                var isCurrent = colKey.indexOf("current") !== -1;
                var isVoltage = colKey.indexOf("voltage") !== -1;

                var seriesName = getLabelByKey(colKey);
                var series = null;

                for (var sIdx = 0; sIdx < chartView.count; sIdx++) {
                    if (chartView.series(sIdx).name === seriesName) {
                        series = chartView.series(sIdx);
                        break;
                    }
                }

                if (!series) {
                    series = chartView.createSeries(ChartView.SeriesTypeLine, seriesName);
                    if (!series) continue;

                    series.axisX = axisX;

                    if (isCurrent) {
                        series.axisYRight = axisYCurrent;
                    } else if (isVoltage) {
                        series.axisY = axisYVoltage;
                    } else {
                        series.axisY = axisYTemp;
                    }
                    series.width = 2;
                }

                series.visible = true;

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

            if (!dataFound) {
                axisYTemp.visible = false;
                axisYVoltage.visible = false;
                axisYCurrent.visible = false;
                messagePopup.message = "查询成功，但在设定的时间范围内没有数据。";
                messagePopup.isError = false;
                messagePopup.open();
                return;
            }

            axisYTemp.visible = hasTemp;
            if (hasTemp) {
                var marginT = Math.max(5, (maxTemp - (-15)) * 0.1);
                axisYTemp.min = -15;
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

            for (var q = 0; q < chartView.count; q++) {
                var visSeries = chartView.series(q);
                if (visSeries && visSeries.visible) {
                    for (var n = 0; n < columnsModel.count; n++) {
                        if (columnsModel.get(n).label === visSeries.name) {
                            columnsModel.setProperty(n, "lineColor", visSeries.color.toString());
                            break;
                        }
                    }
                }
            }
        }
    }

    // 分离带有目标回路 ID 前缀的 Key (如 "1|test_loop_current")
    function getLabelByKey(key) {
        var parts = key.split("|");
        var actualKey = parts.length === 2 ? parts[1] : key;
        for (var i = 0; i < columnsModel.count; i++) {
            if (columnsModel.get(i).key === actualKey) return columnsModel.get(i).label;
        }
        return key;
    }

    MessagePopup { id: messagePopup }
}
