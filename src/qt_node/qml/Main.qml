import QtQuick
import QtQuick.Controls
import QtQuick.Layouts

import qt_node // Import the module containing ROSProxy and QtNodeConstants

Window {
    id: mainWindow
    width: 800
    height: 600
    visible: true
    title: qsTr("Qt + ROS 2 控制面板")

    // A Popup for showing feedback messages to the user
    Popup {
            id: feedbackPopup
            width: 300
            height: 180 // 稍微增加一点高度以容纳按钮
            modal: true
            focus: true
            // 【修正】修正了原始代码中的拼写错误 Outsise -> Outside
            closePolicy: Popup.CloseOnEscape | Popup.CloseOnOutsideClicked
            padding: 20
            anchors.centerIn: parent

            // 【改动】使用 ColumnLayout 来垂直排列标签和按钮
            ColumnLayout {
                anchors.fill: parent
                spacing: 15 // 在标签和按钮之间增加一些间距

                Label {
                    id: feedbackLabel
                    // 【改动】使用布局属性，而不是 anchors
                    Layout.fillWidth: true
                    wrapMode: Text.WordWrap
                    horizontalAlignment: Text.AlignHCenter
                    verticalAlignment: Text.AlignVCenter
                }

                // --- 【新增】关闭按钮 ---
                Button {
                    text: qsTr("关闭")
                    Layout.alignment: Qt.AlignHCenter // 将按钮在水平方向上居中

                    // 当按钮被点击时，调用 popup 的 close() 方法
                    onClicked: {
                        feedbackPopup.close()
                    }
                }
            }
        }

    // Main layout using Flickable for scrollability
    Flickable {
        anchors.fill: parent
        contentHeight: mainLayout.implicitHeight
        flickableDirection: Flickable.VerticalFlick

        ColumnLayout {
            id: mainLayout
            width: parent.width
            spacing: 20

            // ===================================
            //       1. ROS 数据显示区 (无改动)
            // ===================================
            GroupBox {
                // ... (无改动) ...
            }

            // ===================================
            //       2. ROS 命令发送区 (无改动)
            // ===================================
            GroupBox {
                // ... (无改动) ...
            }

            // ===================================
            //       3. ROS 参数设置区 (核心改动)
            // ===================================
            GroupBox {
                id: settingsGroup
                title: "回路 1 参数设置"
                property int circuitId: 1
                Layout.fillWidth: true

                // --- 【改动】使用 Component.onCompleted 初始化 UI ---
                // 当界面加载完成时，用 C++ 对象的默认值来填充 UI 控件
                Component.onCompleted: {
                    var settings = rosProxy.qmlCircuitSettings;
                    testStartCurrentSpinBox.value = settings.test_loop.start_current_a;
                    testMaxCurrentSpinBox.value = settings.test_loop.max_current_a;
                    testCurrentRangeSpinBox.value = settings.test_loop.current_change_range_percent;
                    testCtRatioSpinBox.value = settings.test_loop.ct_ratio;
                    testCycleCountSpinBox.value = settings.test_loop.cycle_count;
                    testHeatingDurationSpinBox.value = settings.test_loop.heating_duration_min;
                    datePickerPopup.selectedTestDate = settings.test_loop.start_date;
                    testHeatingTimeField.text = Qt.formatTime(settings.test_loop.heating_time, "HH:mm");
                }


                ColumnLayout {
                    // --- 试验回路部分 ---
                    GroupBox {
                        title: "试验回路 (Test Loop)"
                        Layout.fillWidth: true
                        GridLayout {
                            columns: 2
                            Label { text: "起始电流 (A):" }
                            SpinBox {
                                id: testStartCurrentSpinBox; from: 0; to: 1000
                                // 【改动】当值改变时，立即更新 C++ 对象
                                onValueChanged: rosProxy.qmlCircuitSettings.test_loop.start_current_a = Math.round(value)
                            }
                            Label { text: "最大电流 (A):" }
                            SpinBox {
                                id: testMaxCurrentSpinBox; from: 0; to: 1000
                                onValueChanged: rosProxy.qmlCircuitSettings.test_loop.max_current_a = Math.round(value)
                            }
                            Label { text: "电流变化范围 (%):" }
                            SpinBox {
                                id: testCurrentRangeSpinBox; from: 0; to: 100
                                onValueChanged: rosProxy.qmlCircuitSettings.test_loop.current_change_range_percent = Math.round(value)
                            }
                            Label { text: "互感器变比:" }
                            SpinBox {
                                id: testCtRatioSpinBox; from: 1; to: 10000
                                onValueChanged: rosProxy.qmlCircuitSettings.test_loop.ct_ratio = Math.round(value)
                            }
                            Label { text: "循环次数:" }
                            SpinBox {
                                id: testCycleCountSpinBox; from: 1; to: 9999
                                onValueChanged: rosProxy.qmlCircuitSettings.test_loop.cycle_count = Math.round(value)
                            }
                            Label { text: "起始日期:" }
                            TextField {
                                id: testStartDateField
                                // 【改动】绑定到 datePickerPopup 的值
                                text: Qt.formatDate(datePickerPopup.selectedTestDate, "yyyy-MM-dd")
                                readOnly: true
                                MouseArea { anchors.fill: parent; onClicked: datePickerPopup.openFor("test") }
                            }
                            Label { text: "加热时刻 (HH:mm):" }
                            TextField { id: testHeatingTimeField; validator: RegularExpressionValidator { regularExpression: /^([0-1]?[0-9]|2[0-3]):[0-5][0-9]$/ } }
                            Label { text: "加热时长 (分钟):" }
                            SpinBox {
                                id: testHeatingDurationSpinBox; from: 1; to: 1440
                                onValueChanged: rosProxy.qmlCircuitSettings.test_loop.heating_duration_min = Math.round(value)
                            }
                        }
                    }

                    // --- 应用按钮 ---
                    Button {
                        text: "应用回路 " + settingsGroup.circuitId + " 设置"
                        Layout.alignment: Qt.AlignRight
                        onClicked: {
                            // --- 【核心改动】不再创建 JS 对象，而是直接调用 C++ 槽函数 ---

                            // 1. 获取 C++ 对象的引用
                            var settings = rosProxy.qmlCircuitSettings;

                            // 2. 更新那些不能直接绑定的值 (日期和时间)
                            settings.test_loop.start_date = datePickerPopup.selectedTestDate;
                            var timeParts = testHeatingTimeField.text.split(":");
                            settings.test_loop.heating_time = new Date(1970, 0, 1, parseInt(timeParts[0]), parseInt(timeParts[1]));

                            // 3. (可选) 为 ref_loop 和 sample_params 设置值
                            //    现在可以直接在 C++ 对象上设置
                            settings.ref_loop.start_current_a = 1;
                            // ...
                            settings.sample_params.cable_type = "YJLV";
                            // ...

                            // 4. 将配置好的 C++ 对象指针传递给 C++ 槽
                            console.log("将 C++ 对象引用传递给 setCircuitSettings...");
                            rosProxy.setCircuitSettings(settingsGroup.circuitId, settings);
                            console.log("调用已执行。");
                        }
                    }
                }
            }
        }
    }

    // ===================================
    //       4. 后端信号处理
    // ===================================
    Connections {
        target: rosProxy
        function onSettingsUpdateResult(service_name, success, message) {
            console.log("Service '" + service_name + "' responded. Success: " + success + ", Message: " + message);
            feedbackLabel.text = "<b>" + (success ? "成功" : "失败") + "</b><br>" + message;
            feedbackPopup.open();
        }
    }

    // ===================================
    //       5. 弹出式日历控件 (Qt 6)
    // ===================================
    Popup {
        id: datePickerPopup
        width: 300
        height: 320
        modal: true
        focus: true
        closePolicy: Popup.CloseOnEscape | Popup.CloseOnOutsiseClicked

        property date selectedTestDate: new Date()
        property string currentPicker: "" // "test" or "ref"

        function openFor(pickerType) {
            currentPicker = pickerType;
            open();
        }

        ColumnLayout {
            anchors.fill: parent
            DayOfWeekRow { locale: monthGrid.locale; Layout.fillWidth: true }
            MonthGrid {
                id: monthGrid
                month: (datePickerPopup.currentPicker === "test" ? datePickerPopup.selectedTestDate : datePickerPopup.selectedTestDate).getMonth()
                year: (datePickerPopup.currentPicker === "test" ? datePickerPopup.selectedTestDate : datePickerPopup.selectedTestDate).getFullYear()
                locale: Qt.locale("zh_CN")
                onClicked: function(date) {
                    if (datePickerPopup.currentPicker === "test") {
                        datePickerPopup.selectedTestDate = date;
                    }
                    // else if (datePickerPopup.currentPicker === "ref") { ... }
                    datePickerPopup.close();
                }
            }
            RowLayout {
                Layout.alignment: Qt.AlignHCenter
                Button { text: "<"; onClicked: monthGrid.month-- }
                Label { text: Qt.formatDate(new Date(monthGrid.year, monthGrid.month), "yyyy年 M月") }
                Button { text: ">"; onClicked: monthGrid.month++ }
            }
        }
    }
}
