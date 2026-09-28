import QtQuick
import QtCharts
import qt_node 1.0
import "../components"

HistoryPageForm {
    id: page

    ListModel { id: columnsModel }

    property int lastMonitorType: -1

    function generateColumns() {
        if (typeof rosProxy === "undefined" || !rosProxy || !rosProxy.systemStatus) return;

        var monitorType = rosProxy.systemStatus.temp_monitor_type;
        // 如果型号没变且已经生成了列表，则跳过
        if (monitorType === lastMonitorType && columnsModel.count > 0) return;
        lastMonitorType = monitorType;

        var numChannels = (monitorType === 1) ? 32 : 40;

        columnsModel.clear();
        columnsModel.append({ key: "regulator_1_voltage", label: "主调压器电压(V)", lineColor: "" });
        columnsModel.append({ key: "regulator_1_current", label: "主调压器电流(A)", lineColor: "" });
        columnsModel.append({ key: "regulator_2_voltage", label: "辅调压器电压(V)", lineColor: "" });
        columnsModel.append({ key: "regulator_2_current", label: "辅调压器电流(A)", lineColor: "" });
        columnsModel.append({ key: "test_loop_current",   label: "试验回路电流(A)", lineColor: "" });
        columnsModel.append({ key: "ref_loop_current",    label: "模拟回路电流(A)", lineColor: "" });

        for (var i = 1; i <= numChannels; i++) {
            var numStr = (i < 10 ? "0" : "") + i;
            var chIdx = (i - 1) % 8 + 1;
            // 历史查询是针对选择的特定回路，提供“相对采集卡”信息即可（即相对回路的第1~4/5卡）
            var relativeCard = Math.floor((i - 1) / 8) + 1;
            columnsModel.append({ key: "circuit_temp" + numStr, label: "温度" + numStr + " (相对卡" + relativeCard + "通" + chIdx + ")", lineColor: "" });
        }
    }

    Component.onCompleted: {
        generateColumns();
        page.colRepeater.model = columnsModel;
    }

    Connections {
        target: typeof rosProxy !== "undefined" ? rosProxy : null
        function onSystemStatusChanged() {
            generateColumns();
        }
    }

    property int checkedCount: 0

    Connections {
        target: colRepeater
        function onItemAdded(index, item) {
            item.onCheckedChanged.connect(function() {
                var cnt = 0;
                for (var i = 0; i < colRepeater.count; i++) {
                    var childItem = colRepeater.itemAt(i);
                    if (childItem && childItem.checked) cnt++;
                }
                page.checkedCount = cnt;

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

    Connections {
        target: page.queryPanel
        function onQueryClicked() {
            var cid = page.queryPanel.circuitId + 1; // 0->1, 1->2
            var selectedCols = [];

            for (var i = 0; i < colRepeater.count; i++) {
                var item = colRepeater.itemAt(i);
                if (item && item.checked) {
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
            var spanHours = parseInt(page.queryPanel.spanDays) * 24;

            for (var idx = 0; idx < chartView.count; idx++) {
                var s = chartView.series(idx);
                if (s) { s.clear(); s.visible = false; }
            }

            for (var m = 0; m < columnsModel.count; m++) {
                var itemNode = colRepeater.itemAt(m);
                if (itemNode && itemNode.checked) {
                    columnsModel.setProperty(m, "lineColor", "");
                }
            }

            if (typeof rosProxy !== "undefined" && rosProxy) {
                rosProxy.queryHistory(dateStr, "00:00", spanHours, selectedCols);
            }
        }
    }

    Connections {
        target: typeof rosProxy !== "undefined" ? rosProxy : null

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

            axisX.min = startDate; axisX.max = endDate;

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
                    if (isCurrent) series.axisYRight = axisYCurrent;
                    else if (isVoltage) series.axisY = axisYVoltage;
                    else series.axisY = axisYTemp;

                    series.width = 2;
                }

                series.visible = true;

                for (var i = 0; i < pointsArray.length; i++) {
                    var pt = pointsArray[i];
                    series.append(pt.x, pt.y);

                    var val = pt.y;
                    if (isCurrent) { hasCur = true; if (val < minCur) minCur = val; if (val > maxCur) maxCur = val; }
                    else if (isVoltage) { hasVol = true; if (val < minVol) minVol = val; if (val > maxVol) maxVol = val; }
                    else { hasTemp = true; if (val < minTemp) minTemp = val; if (val > maxTemp) maxTemp = val; }
                }
            }

            if (!dataFound) {
                axisYTemp.visible = false; axisYVoltage.visible = false; axisYCurrent.visible = false;
                messagePopup.message = "查询成功，但在设定的时间范围内没有数据。"; messagePopup.isError = false; messagePopup.open();
                return;
            }

            axisYTemp.visible = hasTemp;
            if (hasTemp) { var marginT = Math.max(5, (maxTemp - (-15)) * 0.1); axisYTemp.min = -15; axisYTemp.max = maxTemp + marginT; }
            axisYVoltage.visible = hasVol;
            if (hasVol) { var marginV = maxVol * 0.1; if (marginV === 0) marginV = 10; axisYVoltage.min = 0; axisYVoltage.max = maxVol + marginV; }
            axisYCurrent.visible = hasCur;
            if (hasCur) { var marginC = maxCur * 0.1; if (marginC === 0) marginC = 10; axisYCurrent.min = 0; axisYCurrent.max = maxCur + marginC; }

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
