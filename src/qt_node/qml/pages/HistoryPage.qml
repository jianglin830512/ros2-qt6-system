import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtCharts
import qt_node
import qt.theme 1.0

Rectangle {
        Layout.columnSpan: 2
        color: "transparent"
        border.color: Theme.highlightColor

        // ChartView 是所有图表的容器
        ChartView {
            id: chartView
            anchors.fill: parent
            title: "实时曲线"
            antialiasing: true // 开启抗锯齿，使曲线更平滑
            legend.visible: true // 显示图例
            legend.alignment: Qt.AlignBottom // 将图例放在底部

            // --- 1. 定义数据系列 ---
            // 我们这里使用 LineSeries 来创建一条折线图
            LineSeries {
                id: lineSeries
                name: "温度传感器 A" // 这个名字会显示在图例中
                axisX: axisX // 关联下面的X轴
                axisY: axisY // 关联下面的Y轴
            }

            // --- 2. 定义坐标轴 ---
            // 必须显式地将坐标轴添加到 ChartView 的 axes 列表中
            axes: [
                ValueAxis {
                    id: axisX
                    min: 0       // X轴最小值
                    max: 10      // X轴最大值
                    tickCount: 6 // X轴刻度线数量
                    titleText: "时间 (秒)"
                },
                ValueAxis {
                    id: axisY
                    min: -5      // Y轴最小值
                    max: 5       // Y轴最大值
                    tickCount: 11
                    titleText: "温度 (°C)"
                }
            ]

            // --- 3. 添加静态数据 ---
            // 在组件加载完成后，向 lineSeries 中添加一些点
            Component.onCompleted: {
                lineSeries.append(0, 0);
                lineSeries.append(1, 2.5);
                lineSeries.append(2, 4);
                lineSeries.append(3, 1);
                lineSeries.append(4, -2);
                lineSeries.append(5, -3.5);
                lineSeries.append(6, -1);
                lineSeries.append(7, 3);
                lineSeries.append(8, 2);
                lineSeries.append(9, 4.5);
                lineSeries.append(10, 1);
            }
        }
    }
